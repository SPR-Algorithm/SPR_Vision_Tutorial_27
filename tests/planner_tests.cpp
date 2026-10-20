#include "task/auto_aim/planner/planner.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace {
constexpr double pi = 3.14159265358979323846;
void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string &message) {
  require(std::abs(actual - expected) <= tolerance,
          message + ": got " + std::to_string(actual) + ", expected " + std::to_string(expected));
}
struct Config {
  std::filesystem::path path;
  explicit Config(const std::string &text = "{}\n") {
    static unsigned number = 0;
    path = std::filesystem::temp_directory_path() / ("spr_planner_test_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
      "_" + std::to_string(number++) + ".yaml");
    std::ofstream(path) << text;
  }
  ~Config() { std::error_code ignored; std::filesystem::remove(path, ignored); }
};
auto_aim::Target target(double omega = 0.0, double theta = 0.0,
                        const Eigen::Vector3d &translation = Eigen::Vector3d::Zero()) {
  const std::vector<cv::Point2f> corners{{0, 0}, {30, 0}, {30, 12}, {0, 12}};
  auto_aim::Armor armor(18, 0.99f, cv::Rect(0, 0, 30, 12), corners);
  armor.name = auto_aim::ArmorName::outpost;
  armor.type = auto_aim::ArmorType::small;
  armor.xyz_in_world = Eigen::Vector3d(5.0 - 0.275 * std::cos(theta),
    0.2 - 0.275 * std::sin(theta), 1.0) + translation;
  armor.ypr_in_world = {theta, 0.0, 0.0};
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  auto_aim::Target result(armor, {}, 0.275, 3);
  // Set a deterministic already-estimated state; filtering is covered by
  // target_tests, so planner tests isolate physical prediction and control.
  auto &x = const_cast<tools::ExtendedKalmanFilter &>(result.ekf()).x;
  x[7] = omega; x[9] = -0.102; x[10] = -0.204;
  return result;
}
Eigen::VectorXd at(const auto_aim::Target &value, double seconds) {
  auto x = value.ekf_x();
  for (int position : {0, 2, 4, 6}) x[position] += seconds * x[position + 1];
  return x;
}
void verify_intercept(const auto_aim::Target &value, const auto_aim::Plan &plan,
                      const auto_aim::PlanningContext &context, double speed) {
  require(plan.control && plan.solver_ok, "valid plan must solve");
  require(plan.aim_id >= 0 && plan.aim_id < 3, "physical plate id");
  near(plan.prediction_horizon, context.prediction_delay_s + plan.fly_time, 1e-10,
       "prediction delay excludes flight until intercept");
  const auto predicted = at(value, plan.prediction_horizon);
  const auto expected = value.armor_position(predicted, plan.aim_id);
  require((plan.aim_xyza.head<3>() - expected).norm() < 2e-6, "impact position at image + delay + flight");
  const Eigen::Vector3d relative = expected - context.muzzle_world;
  const tools::Trajectory ballistic(speed, relative.head<2>().norm(), relative.z());
  require(!ballistic.unsolvable, "intercept reachable");
  near(plan.target_pitch, ballistic.pitch, 2e-7, "pitch up positive relative to actual muzzle height");
  near(tools::limit_rad(plan.target_yaw - std::atan2(relative.y(), relative.x())), 0.0, 2e-7,
       "yaw from actual muzzle");
  near(plan.distance, relative.norm(), 2e-6, "distance relative to muzzle");
  near(plan.fly_time, ballistic.fly_time, 2e-6, "flight fixed point");
  require(std::abs(plan.yaw_acc) <= 100.0001f && std::abs(plan.pitch_acc) <= 50.0001f,
          "MPC acceleration bound");
  require(plan.tracking_error < 0.01, "center MPC follows ballistic reference");
}
void test_stationary_rotation_stop_reverse() {
  Config cfg;
  auto_aim::Planner planner(cfg.path.string());
  auto_aim::PlanningContext context;
  context.muzzle_world = {0.3, -0.1, 0.8};
  context.prediction_delay_s = 0.037;
  for (double omega : {0.0, 0.8 * pi, 0.0, -0.8 * pi}) {
    auto value = target(omega);
    const auto before = value.ekf_x();
    const auto result = planner.plan(value, 25.0, context);
    verify_intercept(value, result, context, 25.0);
    require((before - value.ekf_x()).norm() == 0.0, "planning leaves filtered state unchanged");
    require(std::abs(result.yaw_vel) < 3.0, "smooth rotation yaw rate");
  }
  auto stationary = target();
  const auto result = planner.plan(stationary, 25.0, context);
  near(result.yaw_vel, 0.0, 1e-4, "stationary yaw rate");
  near(result.pitch_vel, 0.0, 1e-4, "stationary pitch rate");
  require(result.fire, "stationary visible plate offers fire recommendation");
}
void test_arrival_plate_and_negative_reference_offset() {
  Config cfg;
  auto_aim::Planner planner(cfg.path.string());
  auto value = target(0.8 * pi, 0.8);
  auto_aim::PlanningContext context;
  context.muzzle_world.z() = 0.8;
  context.prediction_delay_s = 0.12;
  const auto result = planner.plan(value, 25.0, context);
  verify_intercept(value, result, context, 25.0);
  // At image time plate 0 is closest to facing; at impact plate 2 takes over.
  require(result.aim_id == 2, "select plate using arrival phase, not image phase");
  const auto &x = value.ekf_x();
  const double expected_angle = tools::limit_rad(x[6] + x[7] * result.prediction_horizon +
                                                result.aim_id * 2.0 * pi / 3.0);
  near(tools::limit_rad(result.aim_xyza[3] - expected_angle), 0.0, 2e-6,
       "no accidental half-second trajectory shift");
  const auto copied = value.ekf_x();
  value.predict(-0.5); // Target intentionally refuses backward prediction.
  require((value.ekf_x() - copied).norm() == 0.0, "Target negative predict unsupported fixture");
  const auto replay = planner.plan(value, 25.0, context);
  near(replay.yaw, result.yaw, 1e-6, "analytic past reference works despite Target negative-dt guard");
}
void test_world_translation_and_muzzle_height() {
  Config cfg;
  auto_aim::Planner planner(cfg.path.string());
  const Eigen::Vector3d translation(13.0, -7.0, 4.0);
  auto value = target(-0.8 * pi, -0.3);
  auto shifted = target(-0.8 * pi, -0.3, translation);
  auto_aim::PlanningContext a, b;
  a.muzzle_world = {0.3, -0.1, 0.8};
  a.prediction_delay_s = 0.031;
  b = a; b.muzzle_world += translation;
  const auto first = planner.plan(value, 25.0, a);
  const auto second = planner.plan(shifted, 25.0, b);
  verify_intercept(value, first, a, 25.0);
  verify_intercept(shifted, second, b, 25.0);
  require(first.aim_id == second.aim_id, "translation does not change selected plate");
  near(first.yaw, second.yaw, 1e-6, "translation invariant command yaw");
  near(first.pitch, second.pitch, 1e-6, "translation invariant command pitch");
  near(first.fly_time, second.fly_time, 1e-8, "translation invariant flight");
  a.muzzle_world.z() = 1.8;
  auto high = planner.plan(target(), 25.0, a);
  require(high.control && high.target_pitch < 0.0f, "muzzle above plate produces downward pitch");
}
void test_yaw_wrap() {
  Config cfg;
  auto_aim::Planner planner(cfg.path.string());
  auto value = target(0.8 * pi, pi - 0.02);
  auto &x = const_cast<tools::ExtendedKalmanFilter &>(value.ekf()).x;
  x[0] = -5.0; x[2] = 0.001;
  auto_aim::PlanningContext context;
  context.muzzle_world.z() = 0.8;
  const auto result = planner.plan(value, 25.0, context);
  verify_intercept(value, result, context, 25.0);
  require(std::abs(result.yaw) <= pi && std::abs(result.yaw_vel) < 3.0,
          "crossing pi does not create 2pi/DT rate spike");
}
void test_offsets_and_acceleration_constraints() {
  // Preserve legacy degree offsets; nested planner settings take precedence.
  Config configured("yaw_offset: 40\npitch_offset: 2\nplanner: {yaw_offset: 3}\n");
  auto_aim::Planner planner(configured.path.string());
  auto value = target();
  const auto result = planner.plan(value, 25.0);
  require(result.control, "legacy settings with nested override remain usable");
  const Eigen::Vector3d relative = result.aim_xyza.head<3>();
  const tools::Trajectory ballistic(25.0, relative.head<2>().norm(), relative.z());
  near(tools::limit_rad(result.target_yaw - std::atan2(relative.y(), relative.x())),
       3.0 * pi / 180.0, 2e-7, "nested yaw offset in degrees");
  near(result.target_pitch - ballistic.pitch, 2.0 * pi / 180.0, 2e-7,
       "legacy pitch offset in degrees, up positive");

  Config constrained("planner: {max_yaw_acc: 0.001, max_pitch_acc: 0.001}\n");
  auto_aim::Planner bounded(constrained.path.string());
  const auto limited = bounded.plan(target(0.8 * pi), 25.0);
  require(!limited.control || (std::abs(limited.yaw_acc) <= 0.001001 &&
          std::abs(limited.pitch_acc) <= 0.001001),
          "saturated MPC either meets bounds or refuses control");
  require(limited.control || (!limited.fire && !limited.solver_ok),
          "saturated solver failure remains safe");
}
void test_safe_failure_and_config_validation() {
  Config cfg;
  auto_aim::Planner planner(cfg.path.string());
  require(!planner.plan(std::optional<auto_aim::Target>{}, 25.0).control, "empty target safe");
  require(!planner.plan(auto_aim::Target{}, 25.0).control, "uninitialized target safe");
  for (double speed : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()})
    require(!planner.plan(target(), speed).control, "invalid speed safe");
  auto far = target();
  const_cast<tools::ExtendedKalmanFilter &>(far.ekf()).x[0] = 50.0;
  auto result = planner.plan(far, 10.0);
  require(!result.control && !result.fire && !result.solver_ok && result.aim_id == -1,
          "unreachable target returns fully safe plan");
  auto_aim::PlanningContext context;
  context.prediction_delay_s = -0.01;
  require(!planner.plan(target(), 25.0, context).control, "negative pipeline delay safe");
  for (const std::string text : {"planner: {Q_yaw: [1]}\n", "R_pitch: [-1]\n",
       "planner: {max_yaw_acc: 0}\n", "planner: {max_iter: 0}\n"}) {
    Config malformed(text);
    bool rejected = false;
    try { auto_aim::Planner bad(malformed.path.string()); }
    catch (const std::exception &) { rejected = true; }
    require(rejected, "invalid solver settings rejected before solving");
  }
  Config exhausted("planner: {max_iter: 1, solver_tolerance: 1.0e-12}\n");
  auto_aim::Planner limited(exhausted.path.string());
  result = limited.plan(target(0.8 * pi), 25.0);
  require(!result.control && !result.fire && !result.solver_ok, "nonconverged solver does not command");
  static_assert(!std::is_copy_constructible_v<auto_aim::Planner>, "solver owner cannot be copied");
  // Repeated construction exercises both solver owners; sanitizers can check
  // complete destruction of TinySolver's separately allocated subobjects.
  for (int repeat = 0; repeat < 8; ++repeat) { auto_aim::Planner owned(cfg.path.string()); }
}
}  // namespace
int main() {
  try {
    test_stationary_rotation_stop_reverse();
    test_arrival_plate_and_negative_reference_offset();
    test_world_translation_and_muzzle_height();
    test_yaw_wrap();
    test_offsets_and_acceleration_constraints();
    test_safe_failure_and_config_validation();
    std::cout << "planner tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "planner tests failed: " << error.what() << '\n';
    return 1;
  }
}
