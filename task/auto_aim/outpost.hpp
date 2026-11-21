#ifndef AUTO_AIM__OUTPOST_HPP
#define AUTO_AIM__OUTPOST_HPP

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

#include "armor.hpp"
#include "prediction.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
// ============================================================================
// OutpostEKF：前哨站（Outpost）扩展卡尔曼滤波 —— 大作业题2 的核心。
// ============================================================================
//
// 【物理结构】
//   前哨站 = 固定主体 + 三块中部装甲板，三块板绕竖直轴匀速旋转：
//     - 三块板在水平面内周向间隔 120°；
//     - 三块板处于不同高度（这样旋转时不会互相撞上）；
//     - 每块板到旋转轴的水平距离（半径 r）相同。
//   旋转过程中，随着视角变化，可见的板会轮流切换。
//
// 【要估计什么】
//   5 维状态向量 x = [cx, cy, cz, θ, ω]：
//     cx, cy, cz —— 前哨站旋转轴中心的世界坐标（单位 m），固定不动；
//     θ          —— 第 0 块装甲板的旋转相位角（rad）；
//     ω          —— 旋转角速度（rad/s），正常约 0.8π ≈ 2.513。
//
// 【为什么这样选状态】
//   打前哨站需要知道"未来某时刻哪块板会转到哪"，这由中心 (cx,cy,cz)、
//   相位 θ 和角速度 ω 唯一决定（半径 r 和三块板高度是已知几何参数）。
//   中心固定、相位匀速变化，所以用"恒定角速度"模型就够了。
//
// 【运动模型（线性）】
//   离散化后（dt 为两帧时间间隔）：
//     cx' = cx,  cy' = cy,  cz' = cz,  θ' = θ + ω·dt,  ω' = ω
//   转移矩阵 F 只在第 4 行第 5 列（θ 对 ω）有一个 dt，其余是单位阵。
//
// 【观测模型（非线性）】
//   观测 = 检测到某块板，PnP 解出它的世界坐标 z = [px, py, pz]。
//   第 k 块板（k=0/1/2）的预测位置为：
//     φ  = θ + k·2π/3                        （第 k 块板的方位角）
//     px = cx + r·cos(φ)
//     py = cy + r·sin(φ)
//     pz = cz + h[k]                          （h[k] 是第 k 块板相对中心的高度）
//   因为含 cos/sin，这是关于 θ 的非线性观测，需要算雅可比，所以用 EKF。
//
// 【关联（哪块板）】
//   每块板方位角相差 120°，用"当前估计的 θ"预测三块板各自的水平位置，
//   离观测最近的那块就是它。高度 h[k] 也参与观测方程（用来修正 cz）。
//
// 【初始化（一次观测即可）】
//   装甲板的朝向 yaw（Armor::ypr_in_world[0]）就是"板面朝外"的方位角 φ。
//   中心在板的后方距离 r 处：
//     cx = px - r·cos(yaw),  cy = py - r·sin(yaw)
//   再约定"第一次看到的那块板就是第 0 块"（θ = yaw），于是 cz = pz - h[0]。
//   角速度 ω 先给一个先验（默认 0），让 EKF 自己收敛。
// ============================================================================
class OutpostEKF
{
public:
  explicit OutpostEKF(const std::string & config_path);

  void reset();
  bool initialized() const { return initialized_; }

  // observations：一帧里所有解算成功的装甲板观测（世界坐标）。
  // timestamp：这帧的采集时刻。
  void update(const std::vector<Armor> & observations, TimePoint timestamp);

  // 预测未来某时刻三块板各自的世界位置（供规划器选目标）。
  Prediction predict(TimePoint query_time) const;

private:
  // ---- 状态维度和下标 ----
  static constexpr int kDim = 5;
  static constexpr int kCX = 0;
  static constexpr int kCY = 1;
  static constexpr int kCZ = 2;
  static constexpr int kTheta = 3;
  static constexpr int kOmega = 4;
  static constexpr int kPlateCount = 3;

  // 给一个观测做关联，返回最可能的板编号（0/1/2）。
  // 关联不上（水平距离超过门限）返回 -1。
  int associate(const Armor & obs) const;

  tools::ExtendedKalmanFilter ekf_; // 前哨站的 EKF（矩阵运算都在这个类里）
  bool initialized_ = false;
  TimePoint last_t_{};              // 上一次 update 的采集时刻

  // ---- 几何参数（来自机械图纸，读 YAML 的 outpost 段）----
  double radius_ = 0.4;                                     // 板中心到旋转轴的水平距离 (m)
  std::array<double, kPlateCount> height_{{0.1, -0.05, -0.2}}; // 三块板相对中心高度 (m)

  // ---- 滤波参数 ----
  double q_ang_acc_ = 0.1;     // 角加速度过程噪声 (rad²/s⁴)
  double r_ = 0.001;           // PnP 位置观测噪声 (m²)
  double p0_center_ = 0.01;    // 初始中心位置方差 (m²)
  double p0_theta_ = 1.0;      // 初始相位方差 (rad²)
  double p0_omega_ = 1.0;      // 初始角速度方差 (rad²/s²)
  double init_omega_ = 0.0;    // 初始角速度先验 (rad/s)
  double max_assoc_dist_ = 0.5; // 关联门限 (m)
};
}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_HPP
