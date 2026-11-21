#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <list>
#include <optional>
#include <string>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim
{
// Tracker 的职责：目标管理。
// 它回答一个问题——"现在我到底在盯哪一个目标？它还在不在？"
//
// 题1：单目标（一块平移装甲板）——挑最高置信度那块喂给 Target。
// 题2：前哨站——把这一帧所有有效观测都喂给 Target（可能同时看到多块板）。
class Tracker
{
public:
  explicit Tracker(const std::string & config_path);

  void reset();
  std::string state() const;

  // 调用者（主循环）负责先用 Solver 把观测解算到世界坐标系。
  // timestamp 使用图像采集时刻；无可用目标时返回 nullopt。
  std::optional<Target> track(const std::list<Armor> & observations, TimePoint timestamp);

private:
  // 从一帧的多个观测里挑一个最靠谱的（题1：取置信度最高的那个）。
  const Armor * pick_best(const std::list<Armor> & observations) const;

  Target target_;                 // 被跟踪的目标（内含卡尔曼滤波器）
  std::string state_ = "lost";    // "lost" / "tracking" / "temporary_lost"

  int temp_lost_count_ = 0;         // 连续多少帧没看到目标
  int max_temp_lost_count_ = 15;    // 题1：超过这个数就彻底丢失、重置
  int outpost_max_temp_lost_count_ = 75; // 题2：前哨站允许丢更久（板会转进转出视野）

  bool outpost_mode_ = false;       // 题2 开关（读 only_outpost）
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
