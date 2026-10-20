#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
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

// Unlike Target::predict(), this read-only CV model also accepts negative
// offsets. The center of the reference horizon is command execution time.
Eigen::VectorXd state_at(const Target &target, double dt) {
  Eigen::VectorXd x = target.ekf_x();
  for (int position : {0, 2, 4, 6}) x[position] += dt * x[position + 1];
  x[6] = tools::limit_rad(x[6]);
  return x;
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
  visibility_limit_ = setting<double>(yaml, "visibility_limit_deg", 45.0) * kPi / 180.0;
  decision_speed_ = setting<double>(yaml, "decision_speed", 4.0);
  low_speed_delay_time_ = setting<double>(yaml, "low_speed_delay_time", 0.0);
  high_speed_delay_time_ = setting<double>(yaml, "high_speed_delay_time", 0.0);
  max_yaw_acc_ = setting<double>(yaml, "max_yaw_acc", 100.0);
  max_pitch_acc_ = setting<double>(yaml, "max_pitch_acc", 50.0);
  positive(fire_thresh_, "fire_thresh");
  positive(visibility_limit_, "visibility_limit_deg");
  if (visibility_limit_ >= kPi / 2.0) throw std::invalid_argument("Planner visibility_limit_deg must be < 90");
  nonnegative(decision_speed_, "decision_speed");
  nonnegative(low_speed_delay_time_, "low_speed_delay_time");
  nonnegative(high_speed_delay_time_, "high_speed_delay_time");
  positive(max_yaw_acc_, "max_yaw_acc");
  positive(max_pitch_acc_, "max_pitch_acc");
  const int max_iter = setting<int>(yaml, "max_iter", 100);
  const double tolerance = setting<double>(yaml, "solver_tolerance", 1e-4);
  if (max_iter < 1 || max_iter > 1000) throw std::invalid_argument("Planner max_iter must be in [1,1000]");
  positive(tolerance, "solver_tolerance");

  auto setup = [&](const char *axis, double max_acc) -> SolverPtr {
    const auto q = setting<std::vector<double>>(yaml, std::string("Q_") + axis, {1000.0, 1.0});
    const auto r = setting<std::vector<double>>(yaml, std::string("R_") + axis, {0.01});
    if (q.size() != 2 || r.size() != 1)
      throw std::invalid_argument("Planner Q must have 2 entries and R must have 1 entry");
    positive(q[0], "Q angle"); nonnegative(q[1], "Q velocity"); positive(r[0], "R");
    Eigen::Matrix2d A; A << 1.0, DT, 0.0, 1.0;
    Eigen::Vector2d B(0.5 * DT * DT, DT);
    Eigen::Matrix2d Q = Eigen::Vector2d(q[0], q[1]).asDiagonal();
    Eigen::Matrix<double, 1, 1> R; R << r[0];
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

Planner::Intercept Planner::intercept(const Target &target, int id, double bullet_speed,
                                     const PlanningContext &context, double time_offset) const {
  Intercept result;
  double flight = 0.0;
  bool converged = false;
  for (int iteration = 0; iteration < 24; ++iteration) {
    const auto x = state_at(target, context.prediction_delay_s + time_offset + flight);
    const auto xyz = target.armor_position(x, id);
    const Eigen::Vector3d relative = xyz - context.muzzle_world;
    const double range = relative.head<2>().norm();
    if (!relative.allFinite() || range < 0.05) throw std::runtime_error("Invalid muzzle-relative range");
    const tools::Trajectory ballistic(bullet_speed, range, relative.z());
    if (ballistic.unsolvable || !std::isfinite(ballistic.pitch) ||
        !std::isfinite(ballistic.fly_time) || ballistic.fly_time <= 0.0 || ballistic.fly_time > 5.0)
      throw std::runtime_error("Unreachable ballistic intercept");
    result.xyza << xyz, tools::limit_rad(x[6] + id * 2.0 * kPi / target.armor_num());
    result.angles << tools::limit_rad(std::atan2(relative.y(), relative.x()) + yaw_offset_),
      ballistic.pitch + pitch_offset_;
    result.distance = relative.norm();
    result.flight = ballistic.fly_time;
    const double center_yaw = std::atan2(x[2] - context.muzzle_world.y(), x[0] - context.muzzle_world.x());
    result.visibility = tools::limit_rad(result.xyza[3] - center_yaw);
    if (std::abs(ballistic.fly_time - flight) < 1e-8) { converged = true; break; }
    flight = ballistic.fly_time;
  }
  if (!converged || !result.angles.allFinite() || std::abs(result.angles[1]) >= kPi / 2.0)
    throw std::runtime_error("Ballistic intercept failed to converge");
  return result;
}

int Planner::choose_aim_id(const Target &target, double bullet_speed,
                           const PlanningContext &context) const {
  int best = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  const double omega = target.ekf_x()[7];
  for (int id = 0; id < target.armor_num(); ++id) {
    try {
      const auto candidate = intercept(target, id, bullet_speed, context, 0.0);
      // Face the muzzle at impact; a small preference for an approaching plate
      // avoids choosing an equally visible plate already moving away.
      const double departing = omega * candidate.visibility > 0.0 ? 0.05 : 0.0;
      const double cost = std::abs(candidate.visibility) + departing;
      if (cost < best_cost) { best_cost = cost; best = id; }
    } catch (const std::runtime_error &) { /* Other plates may remain reachable. */ }
  }
  if (best < 0) throw std::runtime_error("No reachable armor plate");
  return best;
}

Trajectory Planner::get_trajectory(const Target &target, int aim_id, double bullet_speed,
                                   const PlanningContext &context, double yaw0) const {
  // Two extra samples give central-difference velocities at both boundaries.
  Eigen::Matrix<double, 2, HORIZON + 2> angles;
  double previous_yaw = yaw0;
  for (int sample = 0; sample < HORIZON + 2; ++sample) {
    const double offset = (sample - HALF_HORIZON - 1) * DT;
    const auto point = intercept(target, aim_id, bullet_speed, context, offset);
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

bool Planner::solve_axis(TinySolver *solver, const Eigen::MatrixXd &reference,
                         double max_acceleration) {
  if (!reference.allFinite()) return false;
  auto &work = *solver->work;
  // Reset in the new local yaw frame; reusing duals after a plate switch or yaw
  // wrap would carry unrelated constraints into the current optimization.
  work.x = reference;
  work.u.setZero();
  work.v = work.vnew = reference;
  work.z.setZero(); work.znew.setZero();
  work.g.setZero(); work.y.setZero();
  work.Xref = reference;
  work.Uref.setZero();
  tiny_set_x0(solver, reference.col(0));
  update_linear_cost(solver);
  const int status = tiny_solve(solver);
  if (status != 0 || solver->solution->solved != 1 || work.status != 1 ||
      !solver->solution->x.allFinite() || !solver->solution->u.allFinite()) return false;
  return solver->solution->u.cwiseAbs().maxCoeff() <= max_acceleration + 1e-8 &&
    (work.x - solver->solution->x).cwiseAbs().maxCoeff() <= solver->settings->abs_pri_tol;
}

Plan Planner::plan(Target target, double bullet_speed) {
  PlanningContext context;
  if (target.isinit && target.ekf_x().size() == 11)
    context.prediction_delay_s = std::abs(target.ekf_x()[7]) > decision_speed_
      ? high_speed_delay_time_ : low_speed_delay_time_;
  return plan(std::move(target), bullet_speed, context);
}
Plan Planner::plan(std::optional<Target> target, double bullet_speed) {
  return target ? plan(std::move(*target), bullet_speed) : Plan{};
}
Plan Planner::plan(Target target, double bullet_speed, const PlanningContext &context) {
  debug_xyza.setZero();
  const auto x = target.ekf_x();
  if (!target.isinit || x.size() != 11 || !x.allFinite() || target.armor_num() < 1 ||
      target.armor_num() > 4 || !context.muzzle_world.allFinite() ||
      !std::isfinite(context.prediction_delay_s) || context.prediction_delay_s < 0.0 ||
      context.prediction_delay_s > 2.0 || !std::isfinite(bullet_speed) ||
      bullet_speed <= 1.0 || bullet_speed > 100.0) return {};
  try {
    // Lock this physical plate throughout all past/future reference samples.
    const int id = choose_aim_id(target, bullet_speed, context);
    const auto impact = intercept(target, id, bullet_speed, context, 0.0);
    const double yaw0 = impact.angles[0];
    const auto trajectory = get_trajectory(target, id, bullet_speed, context, yaw0);
    if (!solve_axis(yaw_solver_.get(), trajectory.topRows<2>(), max_yaw_acc_) ||
        !solve_axis(pitch_solver_.get(), trajectory.bottomRows<2>(), max_pitch_acc_)) return {};
    const auto &yaw = yaw_solver_->solution;
    const auto &pitch = pitch_solver_->solution;
    Plan result;
    result.control = result.solver_ok = true;
    result.aim_id = id;
    result.aim_xyza = impact.xyza;
    result.distance = impact.distance;
    result.fly_time = impact.flight;
    result.prediction_horizon = context.prediction_delay_s + impact.flight;
    result.aim_visibility_angle = impact.visibility;
    result.target_yaw = static_cast<float>(impact.angles[0]);
    result.target_pitch = static_cast<float>(impact.angles[1]);
    result.yaw = static_cast<float>(tools::limit_rad(yaw->x(0, HALF_HORIZON) + yaw0));
    result.yaw_vel = static_cast<float>(yaw->x(1, HALF_HORIZON));
    result.yaw_acc = static_cast<float>(yaw->u(0, HALF_HORIZON));
    result.pitch = static_cast<float>(pitch->x(0, HALF_HORIZON));
    result.pitch_vel = static_cast<float>(pitch->x(1, HALF_HORIZON));
    result.pitch_acc = static_cast<float>(pitch->u(0, HALF_HORIZON));
    result.tracking_error = std::hypot(
      tools::limit_rad(result.yaw - impact.angles[0]), result.pitch - impact.angles[1]);
    result.fire = result.tracking_error < fire_thresh_ && std::abs(impact.visibility) < visibility_limit_;
    debug_xyza = impact.xyza;
    return result;
  } catch (const std::exception &) {
    return {};  // Unreachable/invalid references never produce actuator commands.
  }
}

int Planner::reprojection_aim_id(const Target &target) const {
  if (!target.isinit || target.ekf_x().size() != 11 || !target.ekf_x().allFinite() ||
      target.armor_num() < 1 || target.armor_num() > 4) return -1;
  // Legacy image overlay helper: choose from the same image-time state.
  // Runtime prediction overlays should use Plan::aim_xyza and its actual muzzle.
  const auto x = target.ekf_x();
  const double center_yaw = std::atan2(x[2], x[0]);
  int best = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int id = 0; id < target.armor_num(); ++id) {
    const double cost = target.is_outpost()
      ? std::abs(tools::limit_rad(x[6] + id * 2.0 * kPi / target.armor_num() - center_yaw))
      : target.armor_position(x, id).head<2>().squaredNorm();
    if (cost < best_cost) { best = id; best_cost = cost; }
  }
  return best;
}
Eigen::Vector4d Planner::reprojection_aim_xyza(const Target &target) const {
  const int id = reprojection_aim_id(target);
  if (id < 0) return Eigen::Vector4d::Zero();
  const auto x = target.ekf_x();
  Eigen::Vector4d result;
  result << target.armor_position(x, id), tools::limit_rad(x[6] + id * 2.0 * kPi / target.armor_num());
  return result;
}
}  // namespace auto_aim
