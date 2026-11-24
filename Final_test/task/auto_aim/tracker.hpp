#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <list>
#include <optional>
#include <string>
#include <vector>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim
{
class Tracker
{
public:
  explicit Tracker(const std::string & config_path);

  void reset();
  std::string state() const;

  // 调用者负责先用 Solver 获得同一世界坐标系下的观测。
  // timestamp 使用图像采集时刻；无可用目标时返回 nullopt。
  std::optional<Target> track(const std::list<Armor> & observations, TimePoint timestamp);

private:
  void select(const std::list<Armor> & observations, std::vector<Armor> & selected) const;

  Target target_;
  std::string state_ = "lost";

  bool only_outpost_ = false;
  int min_detect_count_ = 3;
  int max_lost_count_ = 15;          // 普通目标
  int outpost_max_lost_count_ = 7500;  // 前哨站
  // double max_coast_s_ = 0.30;        // 无观测仍输出的最长时间
  // double reset_gap_s_ = 1.00;        // 帧间隔过大视为时间断裂
  double max_coast_s_ = 10.00;
  double reset_gap_s_ = 10.00;

  int detect_count_ = 0;  // 连续有观测的帧数
  int lost_count_ = 0;    // 连续无观测的帧数
  TimePoint last_time_{};  // 上一帧时刻（判断时间断裂）
  TimePoint last_seen_{};  // 上次有观测的时刻（判断目标新鲜度）
  
  // TODO(学生): 补充目标管理所需的数据。
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
