#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace auto_aim
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double deg = pi / 180.0;

template<typename T>
void read_if_present(const YAML::Node & node, const char * key, T & value)
{
  if (node && node[key]) value = node[key].as<T>();
}

bool finite_plan(const Plan & plan)
{
  return plan.control && plan.solver_ok && plan.aim_id >= 0 && plan.aim_xyza.allFinite() &&
         std::isfinite(plan.distance) && plan.distance > 0.0 &&
         std::isfinite(plan.fly_time) && plan.fly_time > 0.0 &&
         std::isfinite(plan.prediction_horizon) && plan.prediction_horizon >= 0.0 &&
         std::isfinite(plan.target_yaw) && std::isfinite(plan.target_pitch) &&
         std::abs(plan.target_pitch) <= pi / 2.0 &&
         std::isfinite(plan.yaw) && std::isfinite(plan.pitch) &&
         std::isfinite(plan.aim_visibility_angle);
}
}  // namespace

const char * to_string(FireReason reason)
{
  switch (reason) {
    case FireReason::disabled: return "disabled";
    case FireReason::control_disabled: return "control_disabled";
    case FireReason::not_following: return "not_following";
    case FireReason::not_tracking: return "not_tracking";
    case FireReason::not_converged: return "not_converged";
    case FireReason::no_measurement: return "no_measurement";
    case FireReason::stale_frame: return "stale_frame";
    case FireReason::invalid_plan: return "invalid_plan";
    case FireReason::planner_hold: return "planner_hold";
    case FireReason::plate_not_visible: return "plate_not_visible";
    case FireReason::aim_not_aligned: return "aim_not_aligned";
    case FireReason::settling: return "settling";
    case FireReason::cooldown: return "cooldown";
    case FireReason::clock_reset: return "clock_reset";
    case FireReason::fire: return "fire";
  }
  return "unknown";
}

ShooterConfig Shooter::load_config(const std::string & config_path)
{
  const auto yaml = YAML::LoadFile(config_path);
  const auto sim = yaml["sim"];
  const auto shooter = yaml["shooter"];
  ShooterConfig config;
  // The application also passes its runtime enable switch in FireInput.
  read_if_present(sim, "fire", config.fire_enabled);
  double max_age_ms = config.max_frame_age_s * 1000.0;
  read_if_present(sim, "max_frame_age_ms", max_age_ms);
  read_if_present(shooter, "max_frame_age_ms", max_age_ms);
  config.max_frame_age_s = max_age_ms / 1000.0;
  read_if_present(shooter, "fire_enabled", config.fire_enabled);
  double yaw_degrees = config.yaw_tolerance / deg;
  double pitch_degrees = config.pitch_tolerance / deg;
  double visibility_degrees = config.visibility_angle / deg;
  read_if_present(shooter, "yaw_tolerance_deg", yaw_degrees);
  read_if_present(shooter, "pitch_tolerance_deg", pitch_degrees);
  read_if_present(shooter, "visibility_angle_deg", visibility_degrees);
  config.yaw_tolerance = yaw_degrees * deg;
  config.pitch_tolerance = pitch_degrees * deg;
  config.visibility_angle = visibility_degrees * deg;
  double interval_ms = config.fire_interval_s * 1000.0;
  double settle_ms = config.settle_time_s * 1000.0;
  read_if_present(shooter, "fire_interval_ms", interval_ms);
  read_if_present(shooter, "settle_time_ms", settle_ms);
  config.fire_interval_s = interval_ms / 1000.0;
  config.settle_time_s = settle_ms / 1000.0;
  return config;
}

Shooter::Shooter(const std::string & config_path) : Shooter(load_config(config_path)) {}

Shooter::Shooter(const ShooterConfig & config) : config_(config)
{
  const auto positive_angle = [](double value) {
    return std::isfinite(value) && value > 0.0 && value <= pi / 2.0;
  };
  if (!positive_angle(config_.yaw_tolerance) || !positive_angle(config_.pitch_tolerance) ||
      !positive_angle(config_.visibility_angle) ||
      !std::isfinite(config_.max_frame_age_s) || config_.max_frame_age_s <= 0.0 ||
      !std::isfinite(config_.fire_interval_s) || config_.fire_interval_s < 0.05 ||
      !std::isfinite(config_.settle_time_s) || config_.settle_time_s < 0.0) {
    throw std::invalid_argument("Invalid shooter configuration (fire interval must be >= 50 ms)");
  }
}

void Shooter::reset()
{
  stable_since_.reset();
  aim_id_ = -1;
}

FireDecision Shooter::update(const FireInput & input)
{
  FireDecision result;
  if (last_update_ && input.now < *last_update_) {
    // A restarted/rebased clock must not inherit a prior launch permission.
    reset();
    last_update_ = input.now;
    last_fire_ = input.now;
    result.reason = FireReason::clock_reset;
    result.cooldown_remaining_s = config_.fire_interval_s;
    return result;
  }
  last_update_ = input.now;
  if (last_fire_) {
    const double elapsed = std::chrono::duration<double>(input.now - *last_fire_).count();
    result.cooldown_remaining_s = std::max(0.0, config_.fire_interval_s - elapsed);
  }
  if (std::isfinite(input.gun_yaw) && std::isfinite(input.plan.target_yaw)) {
    result.yaw_error = std::remainder(input.plan.target_yaw - input.gun_yaw, 2.0 * pi);
  }
  if (std::isfinite(input.gun_pitch) && std::isfinite(input.plan.target_pitch)) {
    result.pitch_error = input.plan.target_pitch - input.gun_pitch;
  }
  auto hold = [&](FireReason reason) {
    reset();
    result.reason = reason;
    return result;
  };

  if (!config_.fire_enabled || !input.fire_enabled) return hold(FireReason::disabled);
  if (!input.control_enabled) return hold(FireReason::control_disabled);
  if (!input.following) return hold(FireReason::not_following);
  if (!input.tracking) return hold(FireReason::not_tracking);
  if (!input.converged) return hold(FireReason::not_converged);
  if (!input.measurement) return hold(FireReason::no_measurement);
  if (!std::isfinite(input.frame_age_s) || input.frame_age_s < 0.0 ||
      input.frame_age_s > config_.max_frame_age_s) return hold(FireReason::stale_frame);
  if (!finite_plan(input.plan) || !std::isfinite(input.gun_yaw) ||
      !std::isfinite(input.gun_pitch)) return hold(FireReason::invalid_plan);
  if (!input.plan.fire) return hold(FireReason::planner_hold);
  if (std::abs(std::remainder(input.plan.aim_visibility_angle, 2.0 * pi)) >
      config_.visibility_angle) return hold(FireReason::plate_not_visible);
  if (std::abs(result.yaw_error) > config_.yaw_tolerance ||
      std::abs(result.pitch_error) > config_.pitch_tolerance) {
    return hold(FireReason::aim_not_aligned);
  }

  if (!stable_since_ || aim_id_ != input.plan.aim_id) {
    stable_since_ = input.now;
    aim_id_ = input.plan.aim_id;
  }
  result.stable_time_s = std::chrono::duration<double>(input.now - *stable_since_).count();
  if (result.stable_time_s + 1e-12 < config_.settle_time_s) {
    result.reason = FireReason::settling;
    return result;
  }
  if (result.cooldown_remaining_s > 1e-12) {
    result.reason = FireReason::cooldown;
    return result;
  }

  last_fire_ = input.now;
  result.fire = true;
  result.reason = FireReason::fire;
  result.cooldown_remaining_s = config_.fire_interval_s;
  return result;
}
}  // namespace auto_aim
