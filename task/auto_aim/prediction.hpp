#ifndef AUTO_AIM__PREDICTION_HPP
#define AUTO_AIM__PREDICTION_HPP

#include <Eigen/Dense>
#include <chrono>
#include <vector>

namespace auto_aim
{
// 统一的时间点类型：用 steady_clock（单调时钟），
// 采集、滤波、预测都共用这一个时钟，互相之间才能直接相减算时间差。
using TimePoint = std::chrono::steady_clock::time_point;

// 控制与可视化使用的输出格式，不规定滤波器的内部状态。
// 也就是说：外面（规划器）只看 Prediction / PredictedArmor 这两个结构，
// 完全不管你在内部把状态存成 6 维还是 5 维、怎么排列。
struct PredictedArmor
{
  // 与输入观测相同的世界坐标系，位置单位 m。
  // xyza 是一个 4 维向量：[x, y, z, yaw]。
  // 前三个是位置（米），最后一个是装甲板朝向 yaw（弧度）。
  // 朝向沿用 Solver 写入的 Armor::ypr_in_world[0] 约定。
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();

  // 你自己维护的关联编号，默认 -1 表示无效。
  // 题1 只有一个目标，直接填 0；题2 前哨站用 0/1/2 表示三块板。
  int id = -1;
};

struct Prediction
{
  bool valid = false;                 // 是否有效。false 表示给不出预测
  std::vector<PredictedArmor> armors; // 候选装甲板列表
};
}  // namespace auto_aim

#endif  // AUTO_AIM__PREDICTION_HPP
