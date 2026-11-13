#include "tracker.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
  Tracker::Tracker(const std::string &config_path) : target_(config_path) {}

  void Tracker::reset()
  {
    target_.reset();
    state_ = "lost";
    // TODO(学生): 清理自己增加的目标管理数据。
    last_update_time_ = TimePoint{};
    lost_frames_count_ = 0; // debug
  }
  std::string Tracker::state() const
  {
    return state_;
  }

  std::optional<Target> Tracker::track(const std::list<Armor> &observations, TimePoint timestamp)
  {
    // ---------- Không có observation ----------
    if (observations.empty())
    {
      if (state_ == "tracking")
      {
        double time_since_last_seen =
            std::chrono::duration<double>(timestamp - last_update_time_).count();
        if (time_since_last_seen <= max_lost_time_s_)
        {
          return target_; // coasting: bám mù ngắn hạn
        }
        tools::logger()->warn("Tracker: tracking -> lost ({:.2f}s)",
                              time_since_last_seen);
        state_ = "lost";
        target_.reset();
      }
      return std::nullopt;
    }

    // ---------- Có observation ----------
    std::vector<Armor> obs_vec(observations.begin(), observations.end());
    bool updated = target_.update(obs_vec, timestamp);

    if (updated)
    {
      if (state_ != "tracking")
        tools::logger()->info("Tracker: {} -> tracking", state_);
      state_ = "tracking";
      last_update_time_ = timestamp;
      return target_;
    }

    // Có observation nhưng update thất bại (gate loại hết hoặc chưa init)
    if (state_ == "tracking")
    {
      double time_since_last_seen =
          std::chrono::duration<double>(timestamp - last_update_time_).count();
      if (time_since_last_seen <= max_lost_time_s_)
      {
        return target_; // vẫn coasting
      }
      state_ = "lost";
      target_.reset();
      tools::logger()->warn("Tracker: tracking -> lost (gate reject, {:.2f}s)",
                            time_since_last_seen);
    }
    return std::nullopt;
  }
} // namespace auto_aim
