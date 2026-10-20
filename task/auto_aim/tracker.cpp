#include "tracker.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path) : target_(config_path) {}

void Tracker::reset()
{
  target_.reset();
  state_ = "lost";
  // TODO(学生): 清理自己增加的目标管理数据。
}

std::string Tracker::state() const
{
  return state_;
}

std::optional<Target> Tracker::track(const std::list<Armor> & observations, TimePoint timestamp)
{
  // TODO(学生): 管理目标的建立、关联、更新和丢失，并调用 Target 接口。
  (void)observations;
  (void)timestamp;
  return std::nullopt;
}
}  // namespace auto_aim
