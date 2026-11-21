#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <chrono>
#include <optional>
#include <string>

#include "planner/planner.hpp"

namespace auto_aim
{
enum class FireReason
{
  disabled,
  control_disabled,
  not_following,
  not_tracking,
  not_converged,
  no_measurement,
  stale_frame,
  invalid_plan,
  planner_hold,
  plate_not_visible,
  aim_not_aligned,
  settling,
  cooldown,
  clock_reset,
  fire
};

const char * to_string(FireReason reason);

struct ShooterConfig
{
  bool fire_enabled = true;
  double yaw_tolerance = 0.35 * 3.14159265358979323846 / 180.0;
  double pitch_tolerance = 0.2 * 3.14159265358979323846 / 180.0;
  double visibility_angle = 55.0 * 3.14159265358979323846 / 180.0;
  double max_frame_age_s = 0.1;
  double fire_interval_s = 0.08;
  double settle_time_s = 0.02;
};

struct FireInput
{
  Plan plan{};
  bool tracking = false;
  bool converged = false;
  // True only when the current frame supplied an accepted EKF observation.
  bool measurement = false;
  bool following = false;
  bool control_enabled = false;
  bool fire_enabled = false;
  double frame_age_s = 0.0;
  // World-frame muzzle orientation; pitch is positive upwards, in radians.
  double gun_yaw = 0.0;
  double gun_pitch = 0.0;
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
};

struct FireDecision
{
  // A single launch request. Call update once per received camera frame.
  bool fire = false;
  FireReason reason = FireReason::not_tracking;
  double yaw_error = 0.0;
  double pitch_error = 0.0;
  double cooldown_remaining_s = 0.0;
  double stable_time_s = 0.0;
};

class Shooter
{
public:
  explicit Shooter(const ShooterConfig & config = {});
  explicit Shooter(const std::string & config_path);

  FireDecision update(const FireInput & input);
  // Discard accumulated readiness, preserving the last-shot cooldown.
  void reset();
  const ShooterConfig & config() const { return config_; }

private:
  using TimePoint = std::chrono::steady_clock::time_point;
  ShooterConfig config_;
  std::optional<TimePoint> last_update_;
  std::optional<TimePoint> last_fire_;
  std::optional<TimePoint> stable_since_;
  int aim_id_ = -1;

  static ShooterConfig load_config(const std::string & config_path);
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP
