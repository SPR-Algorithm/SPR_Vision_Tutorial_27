#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "armor.hpp"
#include "outpost.hpp"
#include "prediction.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
// Target 是"目标"的统一入口，外面（Tracker / Planner）只跟它打交道。
// 它内部根据配置在两种模型之间切换：
//   1. 题1：6 维匀速（CV）卡尔曼滤波，跟踪一块平移的装甲板。
//   2. 题2：5 维前哨站 EKF（OutpostEKF），估计前哨站中心 + 旋转相位 + 角速度。
// 不管走哪条路，对外接口都是 update() / predict()，规划器不用知道内部细节。
class Target
{
public:
  explicit Target(const std::string & config_path);

  void reset();

  // observations 是已经由 Solver 解算到世界坐标系的观测。
  // timestamp 是这帧观测的采集时刻，不能以处理完成时刻替代。
  void update(const std::vector<Armor> & observations, TimePoint timestamp);

  // 查询指定未来时刻的预测结果，不改变当前滤波状态。
  Prediction predict(TimePoint query_time) const;

private:
  // ====================================================================
  // 题1 状态（6 维匀速模型 Constant Velocity）
  // --------------------------------------------------------------------
  // 状态向量 x 一共 6 维，排列如下：
  //   x[0..2] = 世界坐标下的位置 (x, y, z)，单位：米
  //   x[3..5] = 世界坐标下的速度 (vx, vy, vz)，单位：米/秒
  // 朝向 yaw 单独用一个成员记住，不参与滤波（题1 装甲板只平移不旋转）。
  // ====================================================================
  static constexpr int kDim = 6;

  std::string config_path_;
  tools::ExtendedKalmanFilter ekf_; // 题1 的卡尔曼滤波器
  bool initialized_ = false;
  TimePoint timestamp_{};
  double yaw_ = 0.0;
  bool has_yaw_ = false;

  // —— 题1 滤波参数（从 YAML 的 filter 段读进来）——
  double q_ = 10.0;      // 过程噪声：加速度噪声方差 (m²/s⁴)
  double r_ = 0.001;     // 观测噪声：PnP 位置方差 (m²)
  double p0_pos_ = 0.01; // 初始位置协方差 (m²)
  double p0_vel_ = 1.0;  // 初始速度协方差 (m²/s²)

  // ====================================================================
  // 题2 前哨站 EKF
  // ====================================================================
  bool outpost_mode_ = false; // 配置里 only_outpost=true 时启用
  OutpostEKF outpost_;        // 前哨站滤波器（成员变量，跟题1 的 ekf_ 并列）
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
