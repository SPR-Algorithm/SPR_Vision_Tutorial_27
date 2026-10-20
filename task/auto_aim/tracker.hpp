#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <list>
#include <optional>
#include <string>

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
  Target target_;
  std::string state_ = "lost";

  // TODO(学生): 补充目标管理所需的数据。
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
