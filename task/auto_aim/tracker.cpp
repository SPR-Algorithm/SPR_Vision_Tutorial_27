#include "tracker.hpp"
namespace auto_aim
{
Tracker::Tracker(const std::string & config_path) : target_(config_path) {
  // 构造函数里直接初始化状态为丢失
  state_ = "lost";
  lost_count_ = 0;
}

void Tracker::reset()
{
  target_.reset();
  state_ = "lost";
  lost_count_ = 0; // 清理丢帧计数
}

std::string Tracker::state() const
{
  return state_;
}

std::optional<Target> Tracker::track(const std::list<Armor> & observations, TimePoint timestamp)
{
  // 状态是lost的时候，有新观测就初始化新目标
  if (state_ == "lost") {
    if (!observations.empty()) {
      target_.reset();
      // 把list转成vector喂给Target的update接口
      std::vector<Armor> obs_vec(observations.begin(), observations.end());
      target_.update(obs_vec, timestamp);
      state_ = "tracking";
      lost_count_ = 0;
      return target_;
    }
    return std::nullopt;
  }

  // 状态是tracking的时候
  if (state_ == "tracking") {
    if (!observations.empty()) {
      // 有新观测，直接更新目标，丢帧计数清零
      std::vector<Armor> obs_vec(observations.begin(), observations.end());
      target_.update(obs_vec, timestamp);
      lost_count_ = 0;
      return target_;
    } else {
      // 没有观测，丢帧计数+1
      lost_count_++;
      if (lost_count_ > 15) {
        // 丢帧超过15帧，切回lost状态
        reset();
        return std::nullopt;
      }
      // 丢帧没超过阈值，直接返回当前目标，Target内部会自动用匀速模型外推
      return target_;
    }
  }
  return std::nullopt;
}
}  // namespace auto_aim

