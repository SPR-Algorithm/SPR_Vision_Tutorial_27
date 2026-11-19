#include "tracker.hpp"

#include <vector>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string &config_path) : target_(config_path)
{
  try
  {
    const YAML::Node config = YAML::LoadFile(config_path);
    const YAML::Node cfg = config["tracker"] ? config["tracker"] : config;
    max_lost_time_s_ = cfg["max_lost_time_s"].as<double>(max_lost_time_s_);
    confirm_frames_ = cfg["confirm_frames"].as<int>(confirm_frames_);
  }
  catch (const std::exception &e)
  {
    tools::logger()->warn("Tracker config load failed: {}, using defaults", e.what());
  }
}

void Tracker::reset()
{
  target_.reset();
  state_ = State::Lost;
  last_update_time_ = TimePoint{};
  confirm_count_ = 0;
  last_generation_ = target_.generation();
}

std::string Tracker::state() const
{
  switch (state_)
  {
    case State::Lost: return "lost";
    case State::Detecting: return "detecting";
    case State::Tracking: return "tracking";
    case State::TempLost: return "temp_lost";
  }
  return "lost";
}

void Tracker::set_state(State s, const char *why)
{
  if (s == state_) return;
  state_ = s;
  tools::logger()->info("Tracker: -> {} ({})", state(), why);
}

void Tracker::to_lost(const char *why)
{
  target_.reset();
  confirm_count_ = 0;
  set_state(State::Lost, why);
}

std::optional<Target> Tracker::track(const std::list<Armor> &observations, TimePoint timestamp)
{
  // Quan sát rỗng thì không gọi update; state giữ nguyên, predict() ngoại suy từ lần cập nhật cuối.
  bool updated = false;
  if (!observations.empty())
  {
    const std::vector<Armor> vec(observations.begin(), observations.end());
    updated = target_.update(vec, timestamp);
  }

  // Target có thể tự khởi tạo lại bên trong (gián đoạn dài): coi như bắt đầu xác nhận lại.
  const bool reinitialized = updated && target_.generation() != last_generation_;
  last_generation_ = target_.generation();

  const double since_last = std::chrono::duration<double>(timestamp - last_update_time_).count();

  switch (state_)
  {
    case State::Lost:
      if (updated)
      {
        last_update_time_ = timestamp;
        confirm_count_ = 1;
        set_state(confirm_frames_ <= 1 ? State::Tracking : State::Detecting, "target initialized");
      }
      break;

    case State::Detecting:
      if (updated)
      {
        last_update_time_ = timestamp;
        if (reinitialized) confirm_count_ = 1;
        else if (++confirm_count_ >= confirm_frames_) set_state(State::Tracking, "confirmed");
      }
      else
      {
        to_lost("lost while detecting");  // chưa xác nhận mà mất thì bỏ, tránh nhận nhầm detection giả
      }
      break;

    case State::Tracking:
    case State::TempLost:
      if (updated)
      {
        last_update_time_ = timestamp;
        if (reinitialized)
        {
          confirm_count_ = 1;
          set_state(State::Detecting, "filter re-initialized");
        }
        else
        {
          set_state(State::Tracking, "observation accepted");
        }
      }
      else if (since_last > max_lost_time_s_)
      {
        to_lost("coasting timeout");
      }
      else
      {
        set_state(State::TempLost, "no valid observation");
      }
      break;
  }

  // Target tự reset (filter phân kỳ) thì không còn gì để bám.
  if ((state_ == State::Tracking || state_ == State::TempLost || state_ == State::Detecting) &&
      !target_.is_initialized())
    to_lost("filter reset internally");

  if (state_ == State::Tracking || state_ == State::TempLost) return target_;
  return std::nullopt;
}
}  // namespace auto_aim
