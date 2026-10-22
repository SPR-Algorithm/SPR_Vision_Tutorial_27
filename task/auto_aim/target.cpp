#include "target.hpp"

namespace auto_aim
{
Target::Target(const std::string & config_path) : config_path_(config_path)
{
  initialized_ = false;
  last_state_ = Eigen::Vector<double, 6>::Zero();
}

void Target::reset()
{
  initialized_ = false;
  timestamp_ = TimePoint{};
  last_state_.setZero();
  last_vel_.setZero();
}

void Target::update(const std::vector<Armor> & observations, TimePoint timestamp)
{
  // 第一次观测直接初始化状态
  if (!initialized_) {
    if (!observations.empty()) {
      auto first_obs = observations.front();
      last_state_.head<3>() = first_obs.xyz_in_world;
      last_state_.tail<3>().setZero();
      initialized_ = true;
      timestamp_ = timestamp;
    }
    return;
  }

  // 计算时间差，先匀速外推
  double dt = std::chrono::duration<double>(timestamp - timestamp_).count();
  if (dt > 0.0 && dt < 0.1) {
    last_state_.head<3>() += last_state_.tail<3>() * dt;
  }

  // 有新观测就用观测值修正位置，平滑速度
  if (!observations.empty()) {
    auto obs = observations.front();
    // 位置直接用观测值
    last_state_.head<3>() = obs.xyz_in_world;
    // 用位置差平滑更新速度
    Eigen::Vector3d new_vel = (obs.xyz_in_world - last_state_.head<3>()) / std::max(dt, 0.001);
    last_state_.tail<3>() = 0.7 * last_state_.tail<3>() + 0.3 * new_vel;
  }

  // 更新时间戳
  timestamp_ = timestamp;
}

Prediction Target::predict(TimePoint query_time) const
{
  Prediction pred;
  // 未初始化直接返回无效结果
  if (!initialized_) {
    pred.valid = false;
    return pred;
  }

  // 计算查询时刻和当前时刻的时间差
  double dt = std::chrono::duration<double>(query_time - timestamp_).count();
  if (dt < 0.0 || dt > 1.0) {
    pred.valid = false;
    return pred;
  }

  // 匀速外推装甲板位置
  Eigen::Vector3d extrapolate_pos = last_state_.head<3>() + last_state_.tail<3>() * dt;

  PredictedArmor armor;
  armor.xyza.head<3>() = extrapolate_pos;
  armor.xyza[3] = 0.0;
  armor.id = 0;

  pred.armors.push_back(armor);
  pred.valid = true;
  return pred;
}
}  // namespace auto_aim
