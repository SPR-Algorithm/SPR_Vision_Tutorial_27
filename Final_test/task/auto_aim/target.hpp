#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
using TimePoint = std::chrono::steady_clock::time_point;

// 控制与可视化使用的输出格式，不规定滤波器的内部状态。
struct PredictedArmor
{
  // 与输入观测相同的世界坐标系，位置单位 m。
  // 朝向沿用 Solver 写入的 Armor::ypr_in_world[0] 约定，单位 rad。
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
  // 维护的非负关联编号，默认 -1 表示无效；不是模拟器真值编号。
  int id = -1;
};

struct Prediction
{
  bool valid = false;
  std::vector<PredictedArmor> armors;
};

class Target
{
public:
  explicit Target(const std::string & config_path);

  void reset();

  // observations 应先由调用者通过 Solver 解算到同一世界坐标系。
  // timestamp 是这帧观测的采集时刻，不能以处理完成时刻替代。
  void update(const std::vector<Armor> & observations, TimePoint timestamp);

  // 查询指定未来时刻的预测结果，不改变当前滤波状态。
  Prediction predict(TimePoint query_time) const;

private:
  std::string config_path_;
  tools::ExtendedKalmanFilter ekf_;
  bool initialized_ = false;
  TimePoint timestamp_{};

  // TODO(学生): 按自己的建模方案补充内部数据与方法。
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
