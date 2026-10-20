#include "task/auto_aim/shooter.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
constexpr double pi = 3.14159265358979323846;
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

Clock::time_point at(int milliseconds)
{
  return Clock::time_point{} + std::chrono::milliseconds(milliseconds);
}

auto_aim::FireInput ready(int milliseconds = 0)
{
  auto_aim::FireInput input;
  input.plan.control = input.plan.fire = input.plan.solver_ok = true;
  input.plan.aim_id = 0;
  input.plan.aim_xyza << 5.0, 0.0, 1.0, 0.0;
  input.plan.distance = 5.1;
  input.plan.fly_time = 0.2;
  input.plan.prediction_horizon = 0.21;
  input.plan.target_yaw = 0.2f;
  input.plan.target_pitch = 0.1f;
  input.plan.yaw = input.plan.target_yaw;
  input.plan.pitch = input.plan.target_pitch;
  input.tracking = input.converged = input.measurement = input.following = true;
  input.control_enabled = input.fire_enabled = true;
  input.frame_age_s = 0.03;
  input.gun_yaw = input.plan.target_yaw;
  input.gun_pitch = input.plan.target_pitch;
  input.now = at(milliseconds);
  return input;
}

void expect_hold(auto_aim::Shooter & shooter, const auto_aim::FireInput & input,
                 auto_aim::FireReason reason)
{
  const auto result = shooter.update(input);
  require(!result.fire, std::string("unexpected shot: ") + auto_aim::to_string(reason));
  require(result.reason == reason,
          std::string("expected ") + auto_aim::to_string(reason) + ", got " +
          auto_aim::to_string(result.reason));
}

void test_launch_sequence_and_dropout()
{
  auto_aim::Shooter shooter;
  expect_hold(shooter, ready(), auto_aim::FireReason::settling);
  expect_hold(shooter, ready(10), auto_aim::FireReason::settling);
  require(shooter.update(ready(20)).fire, "aligned target launches after settle time");
  expect_hold(shooter, ready(30), auto_aim::FireReason::cooldown);
  expect_hold(shooter, ready(99), auto_aim::FireReason::cooldown);
  require(shooter.update(ready(100)).fire, "next pulse occurs at configured interval");
  auto input = ready(105);
  input.measurement = false;
  expect_hold(shooter, input, auto_aim::FireReason::no_measurement);
  expect_hold(shooter, ready(110), auto_aim::FireReason::settling);
  expect_hold(shooter, ready(130), auto_aim::FireReason::cooldown);
  require(shooter.update(ready(180)).fire, "recovery preserves the previous-shot cooldown");

  input = ready(190);
  input.tracking = false;
  expect_hold(shooter, input, auto_aim::FireReason::not_tracking);
  expect_hold(shooter, ready(300), auto_aim::FireReason::settling);
  require(shooter.update(ready(320)).fire, "reacquired target needs fresh stable alignment");
  input = ready(400);
  input.plan.aim_id = 1;
  expect_hold(shooter, input, auto_aim::FireReason::settling);
  input.now = at(420);
  require(shooter.update(input).fire, "plate switch earns its own settling interval");
}

void test_gates_and_invalid_input()
{
  auto_aim::ShooterConfig config;
  config.settle_time_s = 0.0;
  auto held = [&](auto_aim::FireInput input, auto_aim::FireReason reason) {
    auto_aim::Shooter shooter(config);
    expect_hold(shooter, input, reason);
  };
  auto input = ready();
  input.fire_enabled = false;
  held(input, auto_aim::FireReason::disabled);
  input = ready(); input.control_enabled = false;
  held(input, auto_aim::FireReason::control_disabled);
  input = ready(); input.following = false;
  held(input, auto_aim::FireReason::not_following);
  input = ready(); input.tracking = false;
  held(input, auto_aim::FireReason::not_tracking);
  input = ready(); input.converged = false;
  held(input, auto_aim::FireReason::not_converged);
  input = ready(); input.measurement = false;
  held(input, auto_aim::FireReason::no_measurement);
  for (double age : {0.100001, -0.001, std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
    input = ready(); input.frame_age_s = age;
    held(input, auto_aim::FireReason::stale_frame);
  }
  input = ready(); input.plan.solver_ok = false;
  held(input, auto_aim::FireReason::invalid_plan);
  input = ready(); input.plan.control = false;
  held(input, auto_aim::FireReason::invalid_plan);
  input = ready(); input.plan.fly_time = -1.0;
  held(input, auto_aim::FireReason::invalid_plan);
  input = ready(); input.gun_yaw = std::numeric_limits<double>::quiet_NaN();
  held(input, auto_aim::FireReason::invalid_plan);
  input = ready(); input.plan.fire = false;
  held(input, auto_aim::FireReason::planner_hold);
  input = ready(); input.plan.aim_visibility_angle = 56.0 * pi / 180.0;
  held(input, auto_aim::FireReason::plate_not_visible);
  input = ready(); input.gun_yaw += 0.02;
  held(input, auto_aim::FireReason::aim_not_aligned);
  input = ready(); input.gun_pitch += 0.02;
  held(input, auto_aim::FireReason::aim_not_aligned);

  auto_aim::Shooter shooter(config);
  input = ready(); input.frame_age_s = 0.1;
  require(shooter.update(input).fire, "freshness boundary remains usable");
}


void test_small_armor_alignment_defaults()
{
  auto_aim::ShooterConfig config;
  config.settle_time_s = 0.0;
  auto_aim::Shooter pitch_gate(config), yaw_gate(config), aligned(config);
  auto input = ready();
  // At 6 m, 0.4 degrees is a 42 mm vertical miss: outside a small
  // armor plate's 28 mm half-height, even before other error sources.
  input.gun_pitch += 0.4 * pi / 180.0;
  expect_hold(pitch_gate, input, auto_aim::FireReason::aim_not_aligned);
  input = ready(); input.gun_yaw += 0.4 * pi / 180.0;
  expect_hold(yaw_gate, input, auto_aim::FireReason::aim_not_aligned);
  input = ready();
  input.gun_pitch += 0.15 * pi / 180.0;
  input.gun_yaw += 0.3 * pi / 180.0;
  require(aligned.update(input).fire, "small-armor default tolerances permit tight alignment");
}

void test_angle_wrap_and_clock_rollback()
{
  auto_aim::ShooterConfig config;
  config.settle_time_s = 0.0;
  auto_aim::Shooter shooter(config);
  auto input = ready(100);
  input.plan.target_yaw = -pi + 0.002;
  input.gun_yaw = pi - 0.002;
  const auto first = shooter.update(input);
  require(first.fire, "yaw wrap near +/-pi does not inhibit aligned shot");
  require(std::abs(first.yaw_error - 0.004) < 1e-6, "reported yaw error is wrapped");
  input.now = at(1);
  expect_hold(shooter, input, auto_aim::FireReason::clock_reset);
  input.now = at(21);
  expect_hold(shooter, input, auto_aim::FireReason::cooldown);
  input.now = at(81);
  require(shooter.update(input).fire, "clock rollback enforces a full new cooldown");
  shooter.reset();
  input.now = at(82);
  expect_hold(shooter, input, auto_aim::FireReason::cooldown);
}

void test_config_and_release_checks()
{
  const auto path = std::filesystem::temp_directory_path() /
    ("shooter_test_" + std::to_string(Clock::now().time_since_epoch().count()) + ".yaml");
  struct RemoveFile {
    std::filesystem::path path;
    ~RemoveFile() { std::error_code ec; std::filesystem::remove(path, ec); }
  } cleanup{path};
  {
    std::ofstream file(path);
    file << "sim:\n  fire: false\n  max_frame_age_ms: 70\n"
            "shooter:\n  yaw_tolerance_deg: 0.8\n  pitch_tolerance_deg: 0.9\n"
            "  visibility_angle_deg: 50\n  fire_interval_ms: 60\n  settle_time_ms: 0\n";
  }
  auto_aim::Shooter loaded(path.string());
  require(!loaded.config().fire_enabled, "sim.fire is read as config gate");
  require(std::abs(loaded.config().yaw_tolerance - 0.8 * pi / 180.0) < 1e-12,
          "configured degrees become radians");
  require(std::abs(loaded.config().max_frame_age_s - 0.07) < 1e-12,
          "sim frame-age config uses milliseconds");
  require(std::abs(loaded.config().fire_interval_s - 0.06) < 1e-12,
          "configured fire interval uses milliseconds");
  expect_hold(loaded, ready(), auto_aim::FireReason::disabled);

  auto_aim::ShooterConfig bad;
  bad.fire_interval_s = 0.01;
  bool rejected = false;
  try { auto_aim::Shooter invalid(bad); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "sub-50 ms fire interval rejected even with NDEBUG");
}
}  // namespace

int main()
{
  try {
    test_launch_sequence_and_dropout();
    test_gates_and_invalid_input();
    test_small_armor_alignment_defaults();
    test_angle_wrap_and_clock_rollback();
    test_config_and_release_checks();
    std::cout << "shooter_tests: all checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "shooter_tests: " << error.what() << '\n';
    return 1;
  }
}
