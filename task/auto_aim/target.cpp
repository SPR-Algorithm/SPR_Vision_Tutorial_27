#include "target.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
// 计算两个 steady_clock 时刻的差，转成"秒"返回。
// 卡尔曼滤波里的 dt 必须用真实的时间差，不能用固定值。
double to_seconds(TimePoint a, TimePoint b)
{
  return std::chrono::duration<double>(a - b).count();
}
}  // namespace

Target::Target(const std::string & config_path)
: config_path_(config_path), outpost_(config_path)
{
  // 从 YAML 里读取滤波参数。读不到就用上面写的默认值。
  try {
    auto yaml = YAML::LoadFile(config_path_);
    if (auto filter = yaml["filter"]) {
      q_ = filter["process_noise"].as<double>(q_);
      r_ = filter["observe_noise"].as<double>(r_);
      p0_pos_ = filter["init_pos_var"].as<double>(p0_pos_);
      p0_vel_ = filter["init_vel_var"].as<double>(p0_vel_);
    }
    // 题2 开关：only_outpost=true 时，Target 走前哨站 EKF 逻辑。
    outpost_mode_ = yaml["only_outpost"].as<bool>(false);
  } catch (const std::exception & e) {
    tools::logger()->warn("Target 读取滤波参数失败，使用默认值: {}", e.what());
  }
}

void Target::reset()
{
  // 把滤波器整个丢掉，回到"从没初始化"的状态。
  ekf_ = tools::ExtendedKalmanFilter{};
  initialized_ = false;
  timestamp_ = TimePoint{};
  yaw_ = 0.0;
  has_yaw_ = false;
  outpost_.reset(); // 题2 的前哨站滤波器也一起重置
}

void Target::update(const std::vector<Armor> & observations, TimePoint timestamp)
{
  // ================================================================
  // 题2：前哨站模式，直接把所有观测转给 OutpostEKF 处理。
  // ================================================================
  if (outpost_mode_) {
    outpost_.update(observations, timestamp);
    return;
  }

  // ================================================================
  // 下面是题1 的原逻辑（6 维匀速卡尔曼）
  // ================================================================

  // 情况1：还没初始化 —— 用第一次有效观测直接建立状态
  if (!initialized_) {
    if (observations.empty()) return;

    const Armor & obs = observations.front();
    if (!obs.xyz_in_world.allFinite()) return;

    // 初始状态：位置 = 观测位置，速度 = 0（一开始假设目标静止）。
    Eigen::Vector<double, kDim> x0 = Eigen::Vector<double, kDim>::Zero();
    x0.head<3>() = obs.xyz_in_world;

    // 初始协方差 P0：位置、速度各自是一个 3x3 的对角块。
    Eigen::Matrix<double, kDim, kDim> P0 = Eigen::Matrix<double, kDim, kDim>::Zero();
    P0.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * p0_pos_;
    P0.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * p0_vel_;

    ekf_ = tools::ExtendedKalmanFilter(x0, P0);
    initialized_ = true;
    timestamp_ = timestamp;
    yaw_ = obs.ypr_in_world[0];
    has_yaw_ = true;
    return;
  }

  // 情况2：已经初始化 —— 先预测步，再更新步
  const double dt = to_seconds(timestamp, timestamp_);
  timestamp_ = timestamp;

  if (!(dt > 0.0)) return;

  // 状态转移矩阵 F：匀速模型，位置 += 速度 * dt
  Eigen::Matrix<double, kDim, kDim> F = Eigen::Matrix<double, kDim, kDim>::Identity();
  F.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * dt;

  // 过程噪声 Q：连续白噪声加速度模型（离散化后）
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  Eigen::Matrix<double, kDim, kDim> Q = Eigen::Matrix<double, kDim, kDim>::Zero();
  Q.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q_ * dt4 / 4.0);
  Q.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q_ * dt3 / 2.0);
  Q.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q_ * dt3 / 2.0);
  Q.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q_ * dt2);

  // 预测步：x <- F*x，P <- F*P*F^T + Q
  ekf_.predict(F, Q);

  // 有观测，就再做更新步
  if (!observations.empty()) {
    const Armor & obs = observations.front();
    if (obs.xyz_in_world.allFinite()) {
      Eigen::Vector3d z = obs.xyz_in_world;

      // 观测矩阵 H：从 6 维状态里"取出"位置这 3 维。
      Eigen::Matrix<double, 3, kDim> H = Eigen::Matrix<double, 3, kDim>::Zero();
      H.leftCols<3>() = Eigen::Matrix3d::Identity();

      Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * r_;

      ekf_.update(z, H, R);

      yaw_ = obs.ypr_in_world[0];
      has_yaw_ = true;
    }
  }
}

Prediction Target::predict(TimePoint query_time) const
{
  // 题2：前哨站模式，直接问 OutpostEKF 要预测。
  if (outpost_mode_) return outpost_.predict(query_time);

  // 题1：没初始化过，给不出任何预测。
  if (!initialized_) return {};

  // 用"当前位置 + 速度 * 时间差"外推出 query_time 时刻的位置。
  double dt = to_seconds(query_time, timestamp_);
  if (dt < 0.0) dt = 0.0;

  const Eigen::VectorXd & x = ekf_.x;
  Eigen::Vector3d pos = x.head<3>() + x.segment<3>(3) * dt;

  PredictedArmor armor;
  armor.xyza << pos, (has_yaw_ ? yaw_ : 0.0);
  armor.id = 0; // 题1 只有一个目标，固定编号 0

  Prediction result;
  result.valid = true;
  result.armors.push_back(armor);
  return result;
}
}  // namespace auto_aim
