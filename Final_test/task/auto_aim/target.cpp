#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <yaml-cpp/yaml.h>

#include "tools/math_tools.hpp"

namespace auto_aim
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kNis3 = 16.27;  // 3 维卡方 99.9%
constexpr double kNis4 = 18.47;  // 4 维卡方 99.9%
constexpr double kAssocM = 0.5;  // 位置关联门限 m
constexpr double kAssocYaw = 0.6;  // 朝向关联门限 rad

double wrap(double a) { return tools::limit_rad(a); }

bool good(const Armor & a)
{
  return a.xyz_in_world.allFinite() && std::isfinite(a.ypr_in_world[0]);
}

}  //namespace


Target::Target(const std::string & config_path) : config_path_(config_path)
{
  loadConfig();
  // TODO(学生): 读取自己的滤波参数。
}

void Target::loadConfig() //从配置文件中读取参数
{
  const auto yaml = YAML::LoadFile(config_path_);
  const auto t = yaml["target"];

  auto get = [&t](const char * key, auto fallback) {
    return (t && t[key]) ? t[key].as<decltype(fallback)>() : fallback;
  };
  auto getv = [&t](const char * key, const std::vector<double> & fallback) {
    return (t && t[key]) ? t[key].as<std::vector<double>>() : fallback;
  };

  is_outpost_ = get("is_outpost", false);
  q_trans_ = get("q_trans", 1.0);
  p0_vel_ = get("P0_vel_var_", 1.0);
  max_dt_ = get("max_dt", 0.5);
  max_horizon_ = get("max_horizon", 1.0);

  q_alpha_ = get("q_alpha", 30.0);
  r_yaw_ = get("r_yaw", 0.08);
  radius_ = get("radius", 0.275);
  center_z_prior_ = get("center_z_prior", 1.146);

  const auto rd = getv("R_trans_diag_", {0.0009, 0.0009, 0.0025});
  if (rd.size() == 3) r_diag_ = Eigen::Vector3d(rd[0], rd[1], rd[2]);
  const auto pp = getv("P0_pos_var_", {0.0025, 0.0025, 0.0025});
  if (pp.size() == 3) p0_pos_ = Eigen::Vector3d(pp[0], pp[1], pp[2]);

  heights_ = getv("plate_heights", {-0.036, 0.0, 0.036});
  const auto ang = getv("plate_angles_deg", {0.0, 120.0, 240.0});
  if (heights_.size() == 3 && ang.size() == 3) {
    angles_.resize(3);
    for (int k = 0; k < 3; ++k) angles_[k] = ang[k] * kPi / 180.0;
  }
}

void Target::reset() //初始化
{
  ekf_ = tools::ExtendedKalmanFilter{};
  initialized_ = false;
  timestamp_ = TimePoint{};

  outpost_ = false;
  last_yaw_ = 0.0;
  // TODO(学生): 清理自己增加的状态。
}

void Target::initTranslation(const Armor & seed, TimePoint t) //平移装甲板
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(6); //状态定义，x,y,z,vx,vy,vz
  x << seed.xyz_in_world.x(), seed.xyz_in_world.y(), seed.xyz_in_world.z(), 0, 0, 0; //seed.xyz_in_world第一帧观测到的世界位置，速度为0

  Eigen::VectorXd d(6); //初始协方差
  d << p0_pos_.x(), p0_pos_.y(), p0_pos_.z(), p0_vel_, p0_vel_, p0_vel_;
  ekf_ = tools::ExtendedKalmanFilter{x, d.asDiagonal()};
  
  /*
  d.asDiagonal() =
  [p0_vel_.x   0     0     0     0    0  ]
  [  0    p0_vel_.y  0     0     0    0  ]
  [  0      0   p0_vel_.z  0     0    0  ]
  [  0      0     0     p0_vel_  0    0  ]
  [  0      0     0     0   p0_vel_   0  ]
  [  0      0     0     0    0   p0_vel_ ]
  */

  last_yaw_ = wrap(seed.ypr_in_world[0]); //记录装甲板的朝向
  outpost_ = false; //判定为平移装甲板
  initialized_ = true; //目标是否建立
  timestamp_ = t; //记录初始时刻
}

void Target::initOutpost(const Armor & seed, TimePoint t) //前哨站
{
  int slot = 0; //板子编号
  double best = std::numeric_limits<double>::infinity(); //预测误差
  for (int k = 0; k < 3; ++k) {
    const double e = std::abs(seed.xyz_in_world.z() - (center_z_prior_ + heights_[k]));
    if (e < best) { best = e; slot = k; }
  }

  const double alpha = wrap(seed.ypr_in_world[0]); //观测到的板子的朝向
  const double theta = wrap(alpha - angles_[slot]); //反解出其相位
  const Eigen::Vector3d c = seed.xyz_in_world -
    Eigen::Vector3d(radius_ * std::cos(alpha), radius_ * std::sin(alpha), heights_[slot]); //计算旋转中心c

  Eigen::VectorXd x = Eigen::VectorXd::Zero(5); //状态定义 x,y,x,θ相位,ω角速度
  x << c.x(), c.y(), c.z(), theta, 0.0; //将旋转中心的位置存进x,届时反推出装甲板的击打位置

  Eigen::VectorXd d(5); //初始化协方差
  d << 0.15 * 0.15, 0.15 * 0.15, 0.15 * 0.15, 0.2 * 0.2, 4.0;
  ekf_ = tools::ExtendedKalmanFilter{x, d.asDiagonal()};
  /*
  d.asDiagonal() =
  [0.15 * 0.15   0     0     0     0  ]
  [  0    0.15 * 0.15  0     0     0  ]
  [  0      0   0.15 * 0.15  0     0  ]
  [  0      0     0     0.2 * 0.2  0  ]
  [  0      0     0     0       4.0   ]
  */

  initialized_ = true; //目标是否建立
  outpost_ = true; //判定为前哨站
  timestamp_ = t; //记录初始时刻
}

// 把状态推进到 dt 之后（预测步）
void Target::predictStep(double dt)
{
  if (outpost_) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
    F(3, 4) = dt;
    /* dentity内置初始矩阵，对角线为1,其余为0
    (F)   cx  cy  cz  θ   ω
    cx' [ 1   0   0   0   0 ]
    cy' [ 0   1   0   0   0 ]
    cz' [ 0   0   1   0   0 ]
    θ'  [ 0   0   0   1  dt ]  
    ω'  [ 0   0   0   0   1 ]
    θ' = θ + ω·dt
    */
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(5, 5);
    Q(0, 0) = Q(1, 1) = Q(2, 2) = 2.0 * dt;  // 中心近似静止
    const double dt2 = dt * dt;
    Q(3, 3) = 0.25 * q_alpha_ * dt2 * dt2;
    Q(3, 4) = Q(4, 3) = 0.5 * q_alpha_ * dt2 * dt;
    Q(4, 4) = q_alpha_ * dt2;
    /* 噪声矩阵
    (Q)   cx    cy    cz    θ        ω
    cx  [ 2dt   0     0     0        0    ]
    cy  [ 0    2dt    0     0        0    ]
    cz  [ 0     0    2dt    0        0    ]
    θ   [ 0     0     0   0.25q·dt⁴  0.5q·dt³]
    ω   [ 0     0     0   0.5q·dt³   q·dt² ]
    */
    ekf_.predict(F, Q); //预测
  } //前哨站
  else {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = F(1, 4) = F(2, 5) = dt;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 3; i < 6; ++i) Q(i, i) = q_trans_ * dt;  // 速度随机游走
    ekf_.predict(F, Q); //预测
  } //装甲板
}

void Target::update(const std::vector<Armor> & observations, TimePoint timestamp)
{
  if (timestamp == TimePoint{}) return;
  // 未初始化：用置信度最高的一块有效观测建目标
  if (!initialized_) {
    const Armor * seed = nullptr; //指针，指向信任度最高的那一块板
    for (const auto & a : observations)
      if (good(a) && (!seed || a.confidence > seed->confidence)) seed = &a; //观测有效 且 （没选中过 或者 置信度更高）
    if (!seed) return; //没有有效目标

    const bool use_outpost = is_outpost_ || seed->name == ArmorName::outpost; //判断模型
    if (use_outpost) initOutpost(*seed, timestamp); //前哨站
    else initTranslation(*seed, timestamp); //平移装甲板
    return;  // 首帧只初始化，下一帧再校正
  }

  // 时间推进
  const double dt = std::chrono::duration<double>(timestamp - timestamp_).count();
  if (dt < -1e-3) return;            // 采集时刻倒流
  if (dt > max_dt_) { reset(); return; }  // 间隔过长，下帧重建
  predictStep(dt); //推进时刻
  timestamp_ = timestamp; //记录新的更新时间

  if (outpost_) updateOutpost(observations); //前哨站更新
  else updateTranslation(observations); //平移更新
  // TODO(学生): 完成平移装甲板的滤波，并自行设计前哨站的 EKF 模型。
  // 根据采集时间推进估计，处理初始化、观测更新及无有效观测的情况。
  // (void)observations;
  // (void)timestamp;
}

// ================= 平移更新（线性观测 [x,y,z]）=================
void Target::updateTranslation(const std::vector<Armor> & obs)
{
  const Eigen::Vector3d pred = ekf_.x.head<3>();
  const Armor * best = nullptr;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & a : obs) {
    if (!good(a)) continue;
    const double dp = (a.xyz_in_world - pred).norm();
    if (dp > kAssocM) continue;
    if (dp < best_cost) { best_cost = dp; best = &a; }
  }
  if (!best) return;

  const Eigen::Vector3d z = best->xyz_in_world;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
  H.block<3, 3>(0, 0).setIdentity();
  const Eigen::Matrix3d R = r_diag_.asDiagonal();
  const auto h = [](const Eigen::VectorXd & x) -> Eigen::VectorXd { return x.head<3>(); };

  if (ekf_.correct(z, H, R, h, [](const auto & a, const auto & b) { return a - b; }, kNis3)) {
    last_yaw_ = wrap(best->ypr_in_world[0]);
  }
}

// ================= 前哨站更新（非线性观测，解析雅可比）=================
void Target::updateOutpost(const std::vector<Armor> & obs)
{
  std::vector<bool> used(3, false);
  for (const auto & a : obs) {
    int slot = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int k = 0; k < 3; ++k) {
      if (used[k]) continue;
      const double angle = ekf_.x[3] + angles_[k];
      const Eigen::Vector3d pk(ekf_.x[0] + radius_ * std::cos(angle),
                               ekf_.x[1] + radius_ * std::sin(angle), ekf_.x[2] + heights_[k]);
      const double dp = (a.xyz_in_world - pk).norm();
      const double dy = std::abs(wrap(a.ypr_in_world[0] - wrap(angle)));
      if (dp > kAssocM || dy > kAssocYaw) continue;
      const double cost = dp + 0.3 * dy;
      if (cost < best_cost) { best_cost = cost; slot = k; }
    }
    if (slot < 0) continue;
    used[slot] = true;

    const double angle = ekf_.x[3] + angles_[slot];
    const Eigen::Vector4d z(a.xyz_in_world.x(), a.xyz_in_world.y(), a.xyz_in_world.z(),
                            wrap(a.ypr_in_world[0]));

    // p_k = c + r[cos a, sin a, 0] + [0,0,h_k],  a = theta + phi_k
    // dp/dc = I3, dp/dtheta = r[-sin a, cos a, 0], dpsi/dtheta = 1
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 5);
    H.block<3, 3>(0, 0).setIdentity();
    H(0, 3) = -radius_ * std::sin(angle);
    H(1, 3) = radius_ * std::cos(angle);
    H(3, 3) = 1.0;

    const Eigen::Vector4d r(r_diag_.x(), r_diag_.y(), r_diag_.z(), r_yaw_ * r_yaw_);
    const Eigen::Matrix4d R = r.asDiagonal();

    const auto h = [this, slot](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      const double a = x[3] + angles_[slot];
      return Eigen::Vector4d(x[0] + radius_ * std::cos(a), x[1] + radius_ * std::sin(a),
                             x[2] + heights_[slot], wrap(a));
    };
    const auto sub = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
      Eigen::VectorXd r = a - b;
      r[3] = wrap(r[3]);
      return r;
    };

    ekf_.correct(z, H, R, h, sub, kNis4);
  }
}

Prediction Target::predict(TimePoint query_time) const
{

  Prediction out;
  if (!initialized_) return out; //检查目标是否创建

  const double dt = std::chrono::duration<double>(query_time - timestamp_).count(); //计算时间差
  if (dt < 0.0 || dt > max_horizon_) return out; //检查目标是否有效

  const Eigen::VectorXd & x = ekf_.x; //读取状态
  // const Eigen::Vector3d p = x.head<3>() + x.tail<3>() * dt;
  
  if (!outpost_) {
    PredictedArmor a;
    a.xyza << x[0] + x[3] * dt, x[1] + x[4] * dt, x[2] + x[5] * dt, last_yaw_;
    a.id = 0;
    out.armors.push_back(a);
  } 
  else {
    const double theta = x[3] + x[4] * dt;
    for (int k = 0; k < 3; ++k) {
      const double angle = theta + angles_[k];
      PredictedArmor a;
      a.xyza << x[0] + radius_ * std::cos(angle), x[1] + radius_ * std::sin(angle),
        x[2] + heights_[k], wrap(angle);
      a.id = k;
      out.armors.push_back(a);
    }
  }
  out.valid = true;
  return out; //标记有效并返回

  // TODO(学生): 用当前估计计算查询时刻的装甲板位置及有效性。
  // 当前占位返回无效结果；不能用最后一帧原始观测冒充预测。
  // (void)query_time;
  // return {};
}
}  // namespace auto_aim
