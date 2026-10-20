#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <utility>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
class Target
{
public:
  ArmorName name = ArmorName::not_armor;
  ArmorType armor_type = ArmorType::small;
  ArmorPriority priority = ArmorPriority::fifth;
  bool jumped = false;
  int last_id = 0;
  bool isinit = false;

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t,
    double radius = 0.2, int armor_num = 4);
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius,
    int armor_num, const Eigen::VectorXd & P0_diagonal);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);
  // A camera frame: every accepted observation uses a different plate, and all
  // assignments/innovations are evaluated against the same predicted state.
  int update(const std::vector<Armor> & armors);

  // Multipliers of observation standard deviations, in measurement order:
  // world bearing yaw, world bearing pitch, range, plate yaw. Defaults to one.
  // This changes R via D*R*D, without relaxing association/NIS/fire gates.
  void set_measurement_stddev_scale(const Eigen::Vector4d & scale);

  // Shared 11D layout: cx vx cy vy z_ref vz theta omega r l h.
  // Outpost: l=dh1, h=dh2, three heights are z_ref, z_ref+l, z_ref+h.
  // Four-plate vehicle: l=r2-r1, h=z2-z1, alternating radius and height.
  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  int armor_num() const { return armor_num_; }
  int update_count() const { return update_count_; }
  bool is_outpost() const { return name == ArmorName::outpost && armor_num_ == 3; }
  const std::vector<std::pair<int, int>> & last_associations() const { return last_associations_; }

  // Read-only model access for simulation and finite-difference verification.
  Eigen::Vector3d armor_position(const Eigen::VectorXd & x, int id) const;
  Eigen::Vector4d observation(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd observation_jacobian(const Eigen::VectorXd & x, int id) const;

  bool diverged() const;
  void update_convergency();
  bool is_converged() const;
  bool convergened();
  bool checkinit();

private:
  int armor_num_ = 0;
  int switch_count_ = 0;
  int update_count_ = 0;
  bool is_converged_ = false;
  double last_dt_ = 0.01;
  Eigen::Vector4d measurement_stddev_scale_ = Eigen::Vector4d::Ones();
  std::array<int, 3> height_observations_{{1, 0, 0}};
  std::array<int, 3> height_order_{{0, 1, 2}};
  bool height_order_initialized_ = false;
  std::vector<std::pair<int, int>> last_associations_;
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  static bool valid_observation(const Armor & armor);
  static Eigen::Vector4d measurement(const Armor & armor);
  Eigen::Matrix4d measurement_noise(const Armor & armor) const;
  static Eigen::VectorXd innovation(const Eigen::VectorXd & a, const Eigen::VectorXd & b);
  void apply_outpost_constraints();
  double plate_height(const Eigen::VectorXd & x, int id) const;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
