#include "outpost.hpp"

#include <cmath>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
// steady_clock 时间差转秒。EKF 的 dt 必须用真实时间差，不能用固定值，
// 因为每帧间隔并不严格相等（会掉帧、会波动）。
double to_seconds(TimePoint a, TimePoint b)
{
  return std::chrono::duration<double>(a - b).count();
}

constexpr double kPi = 3.14159265358979323846;
// 三块板周向间隔 120°，换算成弧度。
constexpr double kPlateSpacing = 2.0 * kPi / 3.0;
}  // namespace

OutpostEKF::OutpostEKF(const std::string & config_path)
{
  try {
    auto yaml = YAML::LoadFile(config_path);
    if (auto outpost = yaml["outpost"]) {
      radius_ = outpost["radius"].as<double>(radius_);
      // plate_height 是 3 个数的序列，逐个读，读不到就保留默认值。
      for (int k = 0; k < kPlateCount; ++k) {
        height_[k] = outpost["plate_height"][k].as<double>(height_[k]);
      }
      q_ang_acc_ = outpost["process_noise_ang_acc"].as<double>(q_ang_acc_);
      r_ = outpost["observe_noise"].as<double>(r_);
      p0_center_ = outpost["init_center_var"].as<double>(p0_center_);
      p0_theta_ = outpost["init_angle_var"].as<double>(p0_theta_);
      p0_omega_ = outpost["init_angvel_var"].as<double>(p0_omega_);
      init_omega_ = outpost["init_angvel"].as<double>(init_omega_);
      max_assoc_dist_ = outpost["max_assoc_distance"].as<double>(max_assoc_dist_);
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("OutpostEKF 读取参数失败，使用默认值: {}", e.what());
  }
}

void OutpostEKF::reset()
{
  // 把滤波器整个丢掉，回到"从没初始化"的状态。
  ekf_ = tools::ExtendedKalmanFilter{};
  initialized_ = false;
  last_t_ = TimePoint{};
}

int OutpostEKF::associate(const Armor & obs) const
{
  const auto & x = ekf_.x;

  // 用当前估计的 θ 预测三块板各自的水平位置，找离观测最近的那块。
  // 只比较水平距离（x,y），因为方位角相差 120° 已经足以区分三块板；
  // 高度差放在观测方程里修正 cz，不参与关联，避免高度参数不准时误关联。
  int best_k = -1;
  double best_d = max_assoc_dist_; // 门限：超过这个水平距离就判"不是前哨站的板"
  for (int k = 0; k < kPlateCount; ++k) {
    const double phi = x[kTheta] + k * kPlateSpacing;
    const double px = x[kCX] + radius_ * std::cos(phi);
    const double py = x[kCY] + radius_ * std::sin(phi);
    const double dx = obs.xyz_in_world.x() - px;
    const double dy = obs.xyz_in_world.y() - py;
    const double d = std::hypot(dx, dy);
    if (d < best_d) {
      best_d = d;
      best_k = k;
    }
  }
  return best_k;
}

void OutpostEKF::update(const std::vector<Armor> & observations, TimePoint timestamp)
{
  // ---- 找第一个"位置和朝向都有效"的观测（初始化用）----
  const Armor * first_valid = nullptr;
  for (const auto & obs : observations) {
    if (obs.xyz_in_world.allFinite() && obs.ypr_in_world.allFinite()) {
      first_valid = &obs;
      break;
    }
  }

  // ============================================================
  // 情况1：还没初始化 —— 用第一次观测直接建立状态
  // ============================================================
  if (!initialized_) {
    if (first_valid == nullptr) return;

    // 装甲板朝向 yaw = 板面朝外的方位角 φ（Solver 解出的 ypr_in_world[0]）。
    // 中心在板的后方距离 r 处，所以：
    //   center = 板位置 - r * [cos(yaw), sin(yaw)]
    const double yaw = first_valid->ypr_in_world[0];
    const double cx = first_valid->xyz_in_world.x() - radius_ * std::cos(yaw);
    const double cy = first_valid->xyz_in_world.y() - radius_ * std::sin(yaw);

    // 约定"第一次看到的那块板就是第 0 块"，所以它的高度取 height_[0]。
    // ⚠️ 已知局限：单帧只能看到 1~2 块板，这里没法知道"第一次看到的那块"到底是三块里
    //    的哪一块（因为中心高度 cz 也未知）。如果它实际的高度不是 height_[0]，那么估计出的
    //    cz 会偏一个常数，z 方向的预测（影响 pitch 瞄准）会偏 ≤ 三块板的高度差。
    //    这个偏差不影响水平面的旋转跟踪（θ/ω/cx/cy 都由 x,y 决定，是对的）。
    //    若要彻底消除：初始化时多看一块板，用两块板的高度差反推出"高度相位偏移"。
    const double cz = first_valid->xyz_in_world.z() - height_[0];

    Eigen::Vector<double, kDim> x0;
    x0 << cx, cy, cz, yaw, init_omega_;

    // 初始协方差 P0：对角矩阵，中心/相位/角速度各自给一个"不确定度"。
    Eigen::Matrix<double, kDim, kDim> P0 = Eigen::Matrix<double, kDim, kDim>::Zero();
    P0(kCX, kCX) = p0_center_;
    P0(kCY, kCY) = p0_center_;
    P0(kCZ, kCZ) = p0_center_;
    P0(kTheta, kTheta) = p0_theta_;
    P0(kOmega, kOmega) = p0_omega_;

    ekf_ = tools::ExtendedKalmanFilter(x0, P0);
    initialized_ = true;
    last_t_ = timestamp;
    return;
  }

  // ============================================================
  // 情况2：已初始化 —— 预测步 + 逐观测更新步
  // ============================================================
  const double dt = to_seconds(timestamp, last_t_);
  last_t_ = timestamp;
  if (!(dt > 0.0)) return; // dt 不合法就返回，避免数值爆炸

  // ---- 预测步：状态转移矩阵 F（恒定角速度）----
  // 只有 θ' = θ + ω·dt 一项不是单位阵。
  Eigen::Matrix<double, kDim, kDim> F = Eigen::Matrix<double, kDim, kDim>::Identity();
  F(kTheta, kOmega) = dt;

  // ---- 过程噪声 Q ----
  // 中心基本不动，只给极小的噪声（机械振动 / 解算漂移）。
  // θ 和 ω 用"连续白噪声角加速度"模型离散化：
  //   Q[θθ] = q·dt⁴/4,  Q[θω] = Q[ωθ] = q·dt³/2,  Q[ωω] = q·dt²
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  Eigen::Matrix<double, kDim, kDim> Q = Eigen::Matrix<double, kDim, kDim>::Zero();
  const double q_center = 1e-6; // 中心位置的过程噪声（非常小）
  Q(kCX, kCX) = q_center * dt;
  Q(kCY, kCY) = q_center * dt;
  Q(kCZ, kCZ) = q_center * dt;
  Q(kTheta, kTheta) = q_ang_acc_ * dt4 / 4.0;
  Q(kTheta, kOmega) = q_ang_acc_ * dt3 / 2.0;
  Q(kOmega, kTheta) = q_ang_acc_ * dt3 / 2.0;
  Q(kOmega, kOmega) = q_ang_acc_ * dt2;

  ekf_.predict(F, Q);

  // ---- 更新步：这一帧可能同时看到多块板，逐块关联 + 更新 ----
  // used[k] 记录第 k 块板是否已被某个观测占用，防止两块观测关联到同一块板。
  bool used[kPlateCount] = {false, false, false};
  for (const auto & obs : observations) {
    if (!obs.xyz_in_world.allFinite()) continue;

    const int k = associate(obs);
    if (k < 0 || used[k]) continue; // 关联不上 / 这块板已被别的观测占用
    used[k] = true;

    // 观测值 z = 这块板的世界坐标（3 维）。
    Eigen::Vector3d z = obs.xyz_in_world;

    // 观测噪声 R：三个位置分量独立、等方差。
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * r_;

    // 观测矩阵 H（3×5）：观测模型 h 对状态 x 的雅可比。
    //   ∂px/∂cx = 1,  ∂px/∂θ = -r·sin(φ)
    //   ∂py/∂cy = 1,  ∂py/∂θ =  r·cos(φ)
    //   ∂pz/∂cz = 1
    // 其余偏导都是 0。
    const double phi = ekf_.x[kTheta] + k * kPlateSpacing;
    Eigen::Matrix<double, 3, kDim> H = Eigen::Matrix<double, 3, kDim>::Zero();
    H(0, kCX) = 1.0;
    H(0, kTheta) = -radius_ * std::sin(phi);
    H(1, kCY) = 1.0;
    H(1, kTheta) = radius_ * std::cos(phi);
    H(2, kCZ) = 1.0;

    // 非线性观测函数 h(x)：给定状态 x，返回第 k 块板的预测位置。
    // 注意：lambda 用传入的参数 x（EKF 内部会传当前状态），不要用 this->ekf_.x。
    auto h = [this, k](const Eigen::VectorXd & x) {
      const double ph = x[kTheta] + k * kPlateSpacing;
      Eigen::VectorXd p(3);
      p << x[kCX] + radius_ * std::cos(ph),
           x[kCY] + radius_ * std::sin(ph),
           x[kCZ] + height_[k];
      return p;
    };

    // 更新步：用观测 z 修正状态（内部会算 h(x) 得到预测，再算残差 z - h(x)）。
    ekf_.update(z, H, R, h);
  }
}

Prediction OutpostEKF::predict(TimePoint query_time) const
{
  if (!initialized_) return {};

  const auto & x = ekf_.x;

  // 把相位外推到 query_time：θ(t) = θ + ω·(t - last_t)。
  double dt = to_seconds(query_time, last_t_);
  if (dt < 0.0) dt = 0.0; // 不往回推（规划器不会问过去，这里只是兜底）
  const double theta_future = x[kTheta] + x[kOmega] * dt;

  // 三块板都算出来，让规划器自己挑"朝向枪口"的那块。
  // 每块板：位置 = 中心 + r·[cos φ, sin φ]，yaw = φ（板面朝外）。
  Prediction result;
  result.valid = true;
  for (int k = 0; k < kPlateCount; ++k) {
    const double phi = theta_future + k * kPlateSpacing;
    PredictedArmor armor;
    armor.xyza << x[kCX] + radius_ * std::cos(phi),
                  x[kCY] + radius_ * std::sin(phi),
                  x[kCZ] + height_[k],
                  phi;
    armor.id = k; // 学生维护的关联编号：0/1/2 表示三块板
    result.armors.push_back(armor);
  }
  return result;
}
}  // namespace auto_aim
