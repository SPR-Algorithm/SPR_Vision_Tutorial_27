#include "target.hpp"

namespace auto_aim
{
Target::Target(const std::string & config_path) : config_path_(config_path)
{
  // TODO(学生): 读取自己的滤波参数。
}

void Target::reset()
{
  ekf_ = tools::ExtendedKalmanFilter{};
  initialized_ = false;
  timestamp_ = TimePoint{};
  // TODO(学生): 清理自己增加的状态。
}

void Target::update(const std::vector<Armor> & observations, TimePoint timestamp)
{
  // TODO(学生): 完成平移装甲板的滤波，并自行设计前哨站的 EKF 模型。
  // 根据采集时间推进估计，处理初始化、观测更新及无有效观测的情况。
  (void)observations;
  (void)timestamp;
}

Prediction Target::predict(TimePoint query_time) const
{
  // TODO(学生): 用当前估计计算查询时刻的装甲板位置及有效性。
  // 当前占位返回无效结果；不能用最后一帧原始观测冒充预测。
  (void)query_time;
  return {};
}
}  // namespace auto_aim
