#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <string>

#include "task/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim {
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;
using Trajectory = Eigen::Matrix<double, 4, HORIZON>;

struct PlanningContext {
  // The acquisition time of the observations last passed to Target::update.
  // Use the same steady clock for capture, filtering and prediction.
  TimePoint observation_time{};
  Eigen::Vector3d muzzle_world = Eigen::Vector3d::Zero();
  // Acquisition -> command execution, excluding bullet flight. The caller
  // includes frame age and actuator delay here; Planner adds flight once.
  double prediction_delay_s = 0.0;
};

struct Plan {
  bool control = false;
  bool fire = false;  // Recommendation only: Shooter must gate actual feedback.
  float target_yaw = 0.0f;
  float target_pitch = 0.0f;
  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float yaw_acc = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float pitch_acc = 0.0f;
  bool solver_ok = false;  // Finite usable iterate; strict convergence is not required.
  int aim_id = -1;
  Eigen::Vector4d aim_xyza = Eigen::Vector4d::Zero();
  double distance = 0.0;
  double fly_time = 0.0;
  double prediction_horizon = 0.0;
  double aim_visibility_angle = 0.0;
  double tracking_error = 0.0;
};

class Planner {
public:
  explicit Planner(const std::string &config_path);
  ~Planner();
  Planner(const Planner &) = delete;
  Planner &operator=(const Planner &) = delete;
  Planner(Planner &&) = delete;
  Planner &operator=(Planner &&) = delete;

  // Prediction is supplied by the student's Target. Planner only handles
  // candidate selection, ballistics and yaw/pitch feedforward control.
  Plan plan(const Target &target, double bullet_speed, const PlanningContext &context);

private:
  struct SolverDeleter { void operator()(TinySolver *solver) const noexcept; };
  using SolverPtr = std::unique_ptr<TinySolver, SolverDeleter>;
  struct Intercept {
    int id = -1;
    Eigen::Vector4d xyza;
    Eigen::Vector2d angles;
    double flight = 0.0;
    double visibility = 0.0;
    double distance = 0.0;
  };

  double yaw_offset_ = 0.0, pitch_offset_ = 0.0;
  double fire_thresh_ = 0.003;
  double max_yaw_acc_ = 100.0, max_pitch_acc_ = 50.0;
  SolverPtr yaw_solver_, pitch_solver_;

  Intercept intercept(const Target &target, double bullet_speed,
                      const PlanningContext &context, double time_offset) const;
  Trajectory get_trajectory(const Target &target, double bullet_speed,
                            const PlanningContext &context, double impact_offset,
                            double yaw0) const;
  static bool solve_axis(TinySolver *solver, const Eigen::MatrixXd &reference);
};
}  // namespace auto_aim
#endif  // AUTO_AIM__PLANNER_HPP
