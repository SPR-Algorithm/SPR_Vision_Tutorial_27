#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim {
namespace {
constexpr double kPi = 3.14159265358979323846;

// Nested planner settings override the original top-level YAML keys.
template <typename T>
T setting(const YAML::Node &yaml, const std::string &key, const T &fallback) {
  if (yaml["planner"] && yaml["planner"][key]) return yaml["planner"][key].as<T>();
  return yaml[key] ? yaml[key].as<T>() : fallback;
}
void positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0)
    throw std::invalid_argument(std::string("Planner requires positive finite ") + name);
}
void nonnegative(double value, const char *name) {
  if (!std::isfinite(value) || value < 0.0)
    throw std::invalid_argument(std::string("Planner requires nonnegative finite ") + name);
}

TimePoint query_time(const PlanningContext &context, double offset) {
  // A centered MPC window can extend before the current observation. Reuse
  // that observation's prediction there rather than requiring a model to
  // extrapolate into the past. All remaining samples request future times.
  const auto elapsed = std::chrono::duration<double>(std::max(0.0, offset));
  return context.observation_time +
    std::chrono::duration_cast<TimePoint::duration>(elapsed);
}
}  // namespace

void Planner::SolverDeleter::operator()(TinySolver *solver) const noexcept {
  if (!solver) return;
  delete solver->solution;
  delete solver->settings;
  delete solver->cache;
  delete solver->work;
  delete solver;
}
Planner::~Planner() = default;

Planner::Planner(const std::string &config_path) {
  const auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = setting<double>(yaml, "yaw_offset", 0.0) * kPi / 180.0;
  pitch_offset_ = setting<double>(yaml, "pitch_offset", 0.0) * kPi / 180.0;
  if (!std::isfinite(yaw_offset_) || !std::isfinite(pitch_offset_))
    throw std::invalid_argument("Planner offsets must be finite degrees");
  fire_thresh_ = setting<double>(yaml, "fire_thresh", 0.003);
  max_yaw_acc_ = setting<double>(yaml, "max_yaw_acc", 100.0);
  max_pitch_acc_ = setting<double>(yaml, "max_pitch_acc", 50.0);
  positive(fire_thresh_, "fire_thresh");
  positive(max_yaw_acc_, "max_yaw_acc");
  positive(max_pitch_acc_, "max_pitch_acc");
  const int max_iter = setting<int>(yaml, "max_iter", 10);
  const double tolerance = setting<double>(yaml, "solver_tolerance", 1e-3);
  if (max_iter < 1 || max_iter > 1000) throw std::invalid_argument("Planner max_iter must be in [1,1000]");
  positive(tolerance, "solver_tolerance");

  auto setup = [&](const char *axis, double max_acc) -> SolverPtr {
    const auto q = setting<std::vector<double>>(yaml, std::string("Q_") + axis,
      std::vector<double>{9e6, 0.0});
    const auto r = setting<std::vector<double>>(yaml, std::string("R_") + axis,
      std::vector<double>{1.0});
    if (q.size() != 2 || r.size() != 1)
      throw std::invalid_argument("Planner Q must have 2 entries and R must have 1 entry");
    positive(q[0], "Q angle"); nonnegative(q[1], "Q velocity"); positive(r[0], "R");
    Eigen::Matrix2d A; A << 1.0, DT, 0.0, 1.0;
    Eigen::Vector2d B(0.0, DT);
    Eigen::Matrix2d Q = Eigen::Vector2d(q[0], q[1]).asDiagonal();
    Eigen::Matrix<double, 1, 1> R; R << r[0];
    // 26's tiny_setup stored Q+rho*I and R+rho*I before building the
    // Riccati cache. Match its effective weights here while keeping the
    // shared TinyMPC implementation unchanged.
    Q.diagonal().array() += 1.0;
    R(0, 0) += 1.0;
    TinySolver *raw = nullptr;
    const int status = tiny_setup(&raw, A, B, Eigen::Vector2d::Zero(), Q, R,
                                  1.0, 2, 1, HORIZON, 0);
    SolverPtr owned(raw);
    if (status != 0 || !raw) throw std::runtime_error("TinyMPC setup failed");
    const auto xmin = Eigen::MatrixXd::Constant(2, HORIZON, -1e8);
    const auto xmax = Eigen::MatrixXd::Constant(2, HORIZON, 1e8);
    const auto umin = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_acc);
    const auto umax = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_acc);
    if (tiny_set_bound_constraints(raw, xmin, xmax, umin, umax) != 0)
      throw std::runtime_error("TinyMPC bound setup failed");
    raw->settings->max_iter = max_iter;
    raw->settings->abs_pri_tol = tolerance;
    raw->settings->abs_dua_tol = tolerance;
    raw->settings->adaptive_rho = 0;  // Fixed cached factorization, bounded runtime.
    return owned;
  };
  yaw_solver_ = setup("yaw", max_yaw_acc_);
  pitch_solver_ = setup("pitch", max_pitch_acc_);
}

Planner::Intercept Planner::intercept(const Target &target, double bullet_speed,
                                     const PlanningContext &context, double time_offset) const {
  const auto prediction = target.predict(query_time(context, time_offset));
  if (!prediction.valid) throw std::runtime_error("Target prediction is unavailable");

  // Choose the nearest front-facing candidate. Plate identities, geometry and
  // motion are entirely supplied by Target, independently of target type.
  const PredictedArmor *chosen = nullptr;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto &armor : prediction.armors) {
    if (armor.id < 0 || !armor.xyza.allFinite()) continue;
    const Eigen::Vector3d relative = armor.xyza.head<3>() - context.muzzle_world;
    const double range = relative.head<2>().norm();
    const double visibility = tools::limit_rad(
      armor.xyza[3] - std::atan2(relative.y(), relative.x()));
    if (range < 0.05 || std::abs(visibility) > kPi / 2.0) continue;
    const double distance = relative.squaredNorm();
    if (distance < best_distance) { chosen = &armor; best_distance = distance; }
  }
  if (!chosen) throw std::runtime_error("No usable predicted armor");
  Intercept result;
  result.id = chosen->id;
  result.xyza = chosen->xyza;
  const Eigen::Vector3d xyz = chosen->xyza.head<3>();
  const Eigen::Vector3d relative = xyz - context.muzzle_world;
  const double range = relative.head<2>().norm();
  if (!relative.allFinite() || range < 0.05) throw std::runtime_error("Invalid muzzle-relative range");
  const tools::Trajectory ballistic(bullet_speed, range, relative.z());
  if (ballistic.unsolvable || !std::isfinite(ballistic.pitch) ||
      !std::isfinite(ballistic.fly_time) || ballistic.fly_time <= 0.0 || ballistic.fly_time > 5.0 ||
      std::abs(ballistic.pitch + pitch_offset_) >= kPi / 2.0)
    throw std::runtime_error("Unreachable ballistic reference");
  result.angles << tools::limit_rad(std::atan2(relative.y(), relative.x()) + yaw_offset_),
    ballistic.pitch + pitch_offset_;
  result.distance = relative.norm();
  result.flight = ballistic.fly_time;
  result.visibility = tools::limit_rad(result.xyza[3] - std::atan2(relative.y(), relative.x()));
  return result;
}

Trajectory Planner::get_trajectory(const Target &target, double bullet_speed,
                                   const PlanningContext &context, double impact_offset,
                                   double yaw0) const {
  // Two extra samples give central-difference velocities at both boundaries.
  Eigen::Matrix<double, 2, HORIZON + 2> angles;
  double previous_yaw = yaw0;
  for (int sample = 0; sample < HORIZON + 2; ++sample) {
    const double offset = (sample - HALF_HORIZON - 1) * DT;
    const auto point = intercept(target, bullet_speed, context, impact_offset + offset);
    const double unwrapped = previous_yaw + tools::limit_rad(point.angles[0] - previous_yaw);
    angles.col(sample) << unwrapped - yaw0, point.angles[1];
    previous_yaw = unwrapped;
  }
  Trajectory result;
  for (int sample = 0; sample < HORIZON; ++sample) {
    const Eigen::Vector2d velocity = (angles.col(sample + 2) - angles.col(sample)) / (2.0 * DT);
    result.col(sample) << angles(0, sample + 1), velocity[0], angles(1, sample + 1), velocity[1];
  }
  return result;
}

bool Planner::solve_axis(TinySolver *solver, const Eigen::MatrixXd &reference) {
  if (!reference.allFinite()) return false;
  // Like 26, retain primal, slack and dual variables across calls. In particular
  // a ten-iteration solve continues the preceding frame's optimization instead
  // of starting over on every image. tiny_solve updates costs during its loop.
  solver->work->Xref = reference;
  tiny_set_x0(solver, reference.col(0));
  tiny_solve(solver);
  return solver->work->x.allFinite() && solver->work->u.allFinite();
}

Plan Planner::plan(const Target &target, double bullet_speed, const PlanningContext &context) {
  if (context.observation_time == TimePoint{} || !context.muzzle_world.allFinite() ||
      !std::isfinite(context.prediction_delay_s) || context.prediction_delay_s < 0.0 ||
      context.prediction_delay_s > 2.0 || !std::isfinite(bullet_speed) ||
      bullet_speed <= 1.0 || bullet_speed > 100.0) return {};
  try {
    // Ask the student's predictor at execution time, estimate flight once and
    // then sample around impact time. Planner never advances a hidden state.
    const auto initial = intercept(target, bullet_speed, context, context.prediction_delay_s);
    const double impact_offset = context.prediction_delay_s + initial.flight;
    const auto impact = intercept(target, bullet_speed, context, impact_offset);
    const double yaw0 = impact.angles[0];
    const auto trajectory = get_trajectory(target, bullet_speed, context, impact_offset, yaw0);
    const bool yaw_ok = solve_axis(yaw_solver_.get(), trajectory.topRows<2>());
    const bool pitch_ok = solve_axis(pitch_solver_.get(), trajectory.bottomRows<2>());
    if (!yaw_ok || !pitch_ok) return {};
    const auto &yaw = *yaw_solver_->work;
    const auto &pitch = *pitch_solver_->work;
    Plan result;
    result.control = result.solver_ok = true;
    result.aim_id = impact.id;
    result.aim_xyza = impact.xyza;
    result.distance = impact.distance;
    result.fly_time = initial.flight;
    result.prediction_horizon = impact_offset;
    result.aim_visibility_angle = impact.visibility;
    result.target_yaw = static_cast<float>(tools::limit_rad(trajectory(0, HALF_HORIZON) + yaw0));
    result.target_pitch = static_cast<float>(trajectory(2, HALF_HORIZON));
    result.yaw = static_cast<float>(tools::limit_rad(yaw.x(0, HALF_HORIZON) + yaw0));
    result.yaw_vel = static_cast<float>(yaw.x(1, HALF_HORIZON));
    result.yaw_acc = static_cast<float>(yaw.u(0, HALF_HORIZON));
    result.pitch = static_cast<float>(pitch.x(0, HALF_HORIZON));
    result.pitch_vel = static_cast<float>(pitch.x(1, HALF_HORIZON));
    result.pitch_acc = static_cast<float>(pitch.u(0, HALF_HORIZON));
    for (float value : {result.target_yaw, result.target_pitch, result.yaw, result.yaw_vel,
                        result.yaw_acc, result.pitch, result.pitch_vel, result.pitch_acc})
      if (!std::isfinite(value)) return {};
    constexpr int shoot_sample = HALF_HORIZON + 2;
    result.tracking_error = std::hypot(trajectory(0, shoot_sample) - yaw.x(0, shoot_sample),
                                      trajectory(2, shoot_sample) - pitch.x(0, shoot_sample));
    result.fire = result.tracking_error < fire_thresh_;
    return result;
  } catch (const std::exception &) {
    return {};  // Unreachable/invalid references never produce actuator commands.
  }
}

}  // namespace auto_aim
