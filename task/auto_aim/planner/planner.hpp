#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>

#include "task/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim {
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;
using Trajectory = Eigen::Matrix<double, 4, HORIZON>;

struct PlanningContext {
  Eigen::Vector3d muzzle_world = Eigen::Vector3d::Zero();
  // Image-state time -> command execution time, excluding bullet flight.
  double prediction_delay_s = 0.0;
  // Optional measured gun state, radians, pitch up positive. The centered
  // reference MPC below supplies feedforward; actuator feedback belongs to the
  // gimbal controller and Shooter's actual-angle gate, not its initial state.
  std::optional<double> yaw, pitch, yaw_rate, pitch_rate;
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
  bool solver_ok = false;
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

  Eigen::Vector4d debug_xyza = Eigen::Vector4d::Zero();
  Plan plan(Target target, double bullet_speed);
  Plan plan(Target target, double bullet_speed, const PlanningContext &context);
  Plan plan(std::optional<Target> target, double bullet_speed);
  int reprojection_aim_id(const Target &target) const;
  Eigen::Vector4d reprojection_aim_xyza(const Target &target) const;

private:
  struct SolverDeleter { void operator()(TinySolver *solver) const noexcept; };
  using SolverPtr = std::unique_ptr<TinySolver, SolverDeleter>;
  struct Intercept {
    Eigen::Vector4d xyza;
    Eigen::Vector2d angles;
    double flight = 0.0;
    double visibility = 0.0;
    double distance = 0.0;
  };

  double yaw_offset_ = 0.0, pitch_offset_ = 0.0;
  double fire_thresh_ = 0.003;
  double visibility_limit_ = 0.7853981633974483;
  double low_speed_delay_time_ = 0.0, high_speed_delay_time_ = 0.0;
  double decision_speed_ = 4.0;
  double max_yaw_acc_ = 100.0, max_pitch_acc_ = 50.0;
  SolverPtr yaw_solver_, pitch_solver_;

  Intercept intercept(const Target &target, int id, double bullet_speed,
                      const PlanningContext &context, double time_offset) const;
  int choose_aim_id(const Target &target, double bullet_speed,
                    const PlanningContext &context) const;
  Trajectory get_trajectory(const Target &target, int aim_id, double bullet_speed,
                            const PlanningContext &context, double yaw0) const;
  static bool solve_axis(TinySolver *solver, const Eigen::MatrixXd &reference,
                         double max_acceleration);
};
}  // namespace auto_aim
#endif  // AUTO_AIM__PLANNER_HPP
