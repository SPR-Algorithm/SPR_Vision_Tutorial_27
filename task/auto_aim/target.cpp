#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr double kOutpostHeightStep = 0.102;
constexpr double kPairGate = 18.467;  // chi-square(4), 99.9%
constexpr double kPi = 3.14159265358979323846;

Eigen::VectorXd default_covariance(bool outpost)
{
  if (outpost) {
    // cx vx cy vy z_ref vz theta omega r dh1 dh2
    Eigen::VectorXd diagonal(11);
    diagonal << 0.25, 0.04, 0.25, 0.04, 0.04, 0.04, 0.09, 9.0, 4e-4, 0.04, 0.04;
    return diagonal;
  }
  Eigen::VectorXd diagonal(11);
  diagonal << 1.0, 64.0, 1.0, 64.0, 0.25, 4.0, 0.4, 100.0, 0.01, 0.01, 0.01;
  return diagonal;
}

void acceleration_noise(Eigen::MatrixXd & Q, int p, int v, double dt, double density)
{
  Q(p, p) += density * dt * dt * dt / 3.0;
  Q(p, v) += density * dt * dt / 2.0;
  Q(v, p) += density * dt * dt / 2.0;
  Q(v, v) += density * dt;
}

double quadratic_cost(
  const Eigen::VectorXd & residual, const Eigen::MatrixXd & H,
  const Eigen::MatrixXd & R, const Eigen::MatrixXd & P)
{
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  S = (0.5 * (S + S.transpose())).eval();
  Eigen::LDLT<Eigen::MatrixXd> factor(S);
  if (!S.allFinite() || factor.info() != Eigen::Success ||
      (factor.vectorD().array() <= 0.0).any()) return std::numeric_limits<double>::infinity();
  const double cost = residual.dot(factor.solve(residual));
  return std::isfinite(cost) && cost >= 0.0 ? cost : std::numeric_limits<double>::infinity();
}
}  // namespace

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num)
: Target(armor, t, radius, armor_num, Eigen::VectorXd()) {}

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius,
  int armor_num, const Eigen::VectorXd & P0_diagonal)
: name(armor.name), armor_type(armor.type), priority(armor.priority), isinit(true),
  armor_num_(armor.name == ArmorName::outpost ? 3 : (armor_num > 0 ? armor_num : 4)), t_(t)
{
  if (!valid_observation(armor) || armor_num_ < 1 || armor_num_ > 4) {
    throw std::invalid_argument("Invalid initial target observation/plate count");
  }
  const bool outpost = is_outpost();
  constexpr int dimension = 11;
  const double r = std::isfinite(radius) && radius > 0.0 ? radius : (outpost ? 0.2765 : 0.2);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dimension);
  x0[0] = armor.xyz_in_world[0] + r * std::cos(armor.ypr_in_world[0]);
  x0[2] = armor.xyz_in_world[1] + r * std::sin(armor.ypr_in_world[0]);
  x0[4] = armor.xyz_in_world[2];
  x0[6] = tools::limit_rad(armor.ypr_in_world[0]);
  x0[8] = r;
  if (outpost) {
    // Plate 0 is the first observed physical plate, not necessarily the upper one.
    // The two relative heights start unknown; their positive covariance allows
    // actual observations to establish the three physical plate heights.
    x0[9] = x0[10] = 0.0;
  }

  Eigen::VectorXd diagonal = default_covariance(outpost);
  Eigen::MatrixXd P0;
  if (P0_diagonal.size() != 0) {
    if (P0_diagonal.size() == dimension) {
      for (int i = 0; i < dimension; ++i) {
        if (std::isfinite(P0_diagonal[i]) && P0_diagonal[i] > 0.0) diagonal[i] = P0_diagonal[i];
      }
    } else {
      throw std::invalid_argument("Target covariance must have 11 entries");
    }
  }
  if (P0.size() == 0) P0 = diagonal.asDiagonal();
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd result = a + b;
    result[6] = tools::limit_rad(result[6]);
    return result;
  };
  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  const double dt = tools::delta_time(t, t_);
  if (!std::isfinite(dt) || dt < 0.0) return;
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  if (!isinit || !std::isfinite(dt) || dt <= 0.0) return;
  // Bound direct-API extrapolation to 10 s and integrate with short substeps.
  // Tracker separately resets after a frame gap above 0.5 s.
  const int steps = std::max(1, static_cast<int>(std::ceil(std::min(dt, 10.0) / 0.05)));
  const double step = std::min(dt, 10.0) / steps;
  last_dt_ = std::min(dt, 0.1);
  for (int s = 0; s < steps; ++s) {
    const int n = static_cast<int>(ekf_.x.size());
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
    for (const int p : {0, 2, 4, 6}) F(p, p + 1) = step;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
    acceleration_noise(Q, 0, 1, step, is_outpost() ? 0.02 : 1.0);
    acceleration_noise(Q, 2, 3, step, is_outpost() ? 0.02 : 1.0);
    acceleration_noise(Q, 6, 7, step, is_outpost() ? 2.0 : 4.0);
    if (is_outpost()) {
      acceleration_noise(Q, 4, 5, step, 0.005);
      Q(8, 8) = 1e-6 * step;
      Q(9, 9) = Q(10, 10) = 1e-5 * step;
    } else {
      acceleration_noise(Q, 4, 5, step, 0.2);
      Q(8, 8) = Q(9, 9) = Q(10, 10) = 1e-5 * step;
    }
    ekf_.predict(F, Q, [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      Eigen::VectorXd next = F * x;
      next[6] = tools::limit_rad(next[6]);
      return next;
    });
  }
  update_convergency();
}

bool Target::valid_observation(const Armor & armor)
{
  return armor.xyz_in_world.allFinite() && armor.ypr_in_world.allFinite() &&
    armor.ypd_in_world.allFinite() && armor.xyz_in_world.norm() > 0.1 &&
    armor.xyz_in_world.norm() < 100.0 && armor.xyz_in_world.head<2>().norm() > 0.05 &&
    armor.ypd_in_world[2] > 0.1;
}

Eigen::Vector4d Target::measurement(const Armor & armor)
{
  // Derive spherical coordinates from the same position so stale cached ypd
  // cannot produce a mutually inconsistent camera observation.
  const Eigen::Vector3d ypd = tools::xyz2ypd(armor.xyz_in_world);
  return {ypd[0], ypd[1], ypd[2], tools::limit_rad(armor.ypr_in_world[0])};
}

void Target::set_measurement_stddev_scale(const Eigen::Vector4d & scale)
{
  if (!scale.allFinite() || (scale.array() <= 0.0).any()) {
    throw std::invalid_argument("target.measurement_stddev_scale must contain positive finite values");
  }
  measurement_stddev_scale_ = scale;
}

Eigen::Matrix4d Target::measurement_noise(const Armor & armor) const
{
  const double d = armor.xyz_in_world.norm();
  const double view = std::abs(tools::limit_rad(
    armor.ypr_in_world[0] - std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0])));
  const double oblique = 1.0 + 0.6 * std::min(view, kPi / 2.0);
  const double bearing_sigma = 0.0015 + 0.0003 * d;
  const double distance_sigma = (0.02 + 0.004 * d * d) * oblique;
  const double orientation_sigma = (0.035 + 0.006 * d * d) * oblique;
  const Eigen::Matrix4d base = Eigen::Vector4d(
    bearing_sigma * bearing_sigma, bearing_sigma * bearing_sigma,
    distance_sigma * distance_sigma, orientation_sigma * orientation_sigma).asDiagonal();
  const Eigen::Matrix4d D = measurement_stddev_scale_.asDiagonal();
  return D * base * D;
}

Eigen::VectorXd Target::innovation(const Eigen::VectorXd & a, const Eigen::VectorXd & b)
{
  Eigen::VectorXd residual = a - b;
  for (Eigen::Index i = 0; i + 3 < residual.size(); i += 4) {
    residual[i] = tools::limit_rad(residual[i]);
    residual[i + 1] = tools::limit_rad(residual[i + 1]);
    residual[i + 3] = tools::limit_rad(residual[i + 3]);
  }
  return residual;
}

void Target::update(const Armor & armor) { update(std::vector<Armor>{armor}); }

int Target::update(const std::vector<Armor> & armors)
{
  last_associations_.clear();
  jumped = false;
  if (!isinit || armors.empty() || diverged()) return 0;
  struct Candidate {
    int input_index;
    Eigen::Vector4d z;
    Eigen::Matrix4d R;
    std::vector<double> costs;
    double minimum_cost;
  };
  const Eigen::VectorXd prior_x = ekf_.x;
  const Eigen::MatrixXd prior_P = ekf_.P;
  std::vector<Eigen::MatrixXd> jacobians;
  std::vector<Eigen::Vector4d> predictions;
  for (int id = 0; id < armor_num_; ++id) {
    jacobians.push_back(observation_jacobian(prior_x, id));
    predictions.push_back(observation(prior_x, id));
  }
  std::vector<Candidate> candidates;
  double best_rejected_nis = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
    const auto & armor = armors[i];
    if (armor.name != name || armor.type != armor_type || !valid_observation(armor)) continue;
    Candidate candidate{i, measurement(armor), measurement_noise(armor), {},
                        std::numeric_limits<double>::infinity()};
    for (int id = 0; id < armor_num_; ++id) {
      const Eigen::VectorXd residual = innovation(candidate.z, predictions[id]);
      double cost = quadratic_cost(residual, jacobians[id], candidate.R, prior_P);
      best_rejected_nis = std::min(best_rejected_nis, cost);
      // A yaw jump to a different physical plate is handled by assignment, not
      // accepted as an arbitrary phase correction even while P0 is broad.
      const auto predicted_xyz = armor_position(prior_x, id);
      if (std::abs(residual[3]) > 1.15 ||
          (armor.xyz_in_world - predicted_xyz).norm() > 1.25) {
        cost = std::numeric_limits<double>::infinity();
      }
      candidate.costs.push_back(cost);
      candidate.minimum_cost = std::min(candidate.minimum_cost, cost);
    }
    if (candidate.minimum_cost <= kPairGate) candidates.push_back(std::move(candidate));
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.minimum_cost < b.minimum_cost;
  });
  // Bound work for a frame with many duplicate detections. Normal 3-plate frames
  // enumerate exactly the 3! complete assignments, plus gated partial matches.
  if (candidates.size() > static_cast<size_t>(2 * armor_num_)) candidates.resize(2 * armor_num_);

  using Assignment = std::vector<std::pair<int, int>>;  // candidate index, physical plate id
  Assignment selected, best;
  double best_cost = std::numeric_limits<double>::infinity();
  auto stack = [&](const Assignment & assignment, Eigen::VectorXd & z, Eigen::MatrixXd & H,
                   Eigen::MatrixXd & R, Eigen::VectorXd & predicted) {
    const int m = static_cast<int>(assignment.size()) * 4;
    z.resize(m); H.resize(m, prior_x.size()); R = Eigen::MatrixXd::Zero(m, m); predicted.resize(m);
    for (int j = 0; j < static_cast<int>(assignment.size()); ++j) {
      const auto [index, id] = assignment[j];
      z.segment<4>(4 * j) = candidates[index].z;
      H.middleRows(4 * j, 4) = jacobians[id];
      R.block<4, 4>(4 * j, 4 * j) = candidates[index].R;
      predicted.segment<4>(4 * j) = predictions[id];
    }
  };
  std::function<void(int, unsigned)> search = [&](int index, unsigned used) {
    if (selected.size() + candidates.size() - index < best.size()) return;
    if (index == static_cast<int>(candidates.size()) ||
        selected.size() == static_cast<size_t>(armor_num_)) {
      if (selected.empty() || selected.size() < best.size()) return;
      Eigen::VectorXd z, predicted;
      Eigen::MatrixXd H, R;
      stack(selected, z, H, R, predicted);
      const double cost = quadratic_cost(innovation(z, predicted), H, R, prior_P);
      if (cost > tools::ExtendedKalmanFilter::nis_threshold(static_cast<int>(z.size()))) return;
      if (selected.size() > best.size() || cost < best_cost) {
        best = selected;
        best_cost = cost;
      }
      return;
    }
    for (int id = 0; id < armor_num_; ++id) {
      if ((used & (1u << id)) || candidates[index].costs[id] > kPairGate) continue;
      selected.emplace_back(index, id);
      search(index + 1, used | (1u << id));
      selected.pop_back();
    }
    search(index + 1, used);
  };
  search(0, 0);
  if (best.empty()) {
    ekf_.record_rejection(best_rejected_nis);
    update_convergency();
    return 0;
  }
  Eigen::VectorXd z, predicted;
  Eigen::MatrixXd H, R;
  stack(best, z, H, R, predicted);
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd result(4 * best.size());
    for (int j = 0; j < static_cast<int>(best.size()); ++j) {
      result.segment<4>(4 * j) = observation(x, best[j].second);
    }
    return result;
  };
  if (!ekf_.correct(z, H, R, h, innovation,
                    tools::ExtendedKalmanFilter::nis_threshold(static_cast<int>(z.size())))) return 0;
  for (const auto [index, id] : best) {
    last_associations_.emplace_back(candidates[index].input_index, id);
    if (is_outpost()) ++height_observations_[id];
  }
  std::sort(last_associations_.begin(), last_associations_.end());
  const int current_id = last_associations_.front().second;
  jumped = current_id != last_id;
  if (jumped) ++switch_count_;
  last_id = current_id;
  update_count_ += static_cast<int>(best.size());
  if (is_outpost()) apply_outpost_constraints();
  update_convergency();
  return static_cast<int>(best.size());
}

void Target::apply_outpost_constraints()
{
  if (!height_order_initialized_ &&
      *std::min_element(height_observations_.begin(), height_observations_.end()) >= 3) {
    std::sort(height_order_.begin(), height_order_.end(), [&](int a, int b) {
      return plate_height(ekf_.x, a) > plate_height(ekf_.x, b);
    });
    const double gap1 = plate_height(ekf_.x, height_order_[0]) - plate_height(ekf_.x, height_order_[1]);
    const double gap2 = plate_height(ekf_.x, height_order_[1]) - plate_height(ekf_.x, height_order_[2]);
    // Do not manufacture a height identity when observations are ambiguous.
    height_order_initialized_ = gap1 > 0.04 && gap1 < 0.18 && gap2 > 0.04 && gap2 < 0.18;
  }
  if (!height_order_initialized_) return;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 11);
  Eigen::VectorXd z = Eigen::VectorXd::Constant(2, kOutpostHeightStep);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
  // Noise scales as 1/dt: changing camera FPS does not strengthen the prior.
  R *= 0.025 * 0.025 / std::clamp(last_dt_, 0.001, 0.1);
  for (int i = 0; i < 2; ++i) {
    // All heights share z_ref and vz, so their differences involve only dh1/dh2.
    const int upper = height_order_[i];
    const int lower = height_order_[i + 1];
    if (upper != 0) H(i, 8 + upper) += 1.0;
    if (lower != 0) H(i, 8 + lower) -= 1.0;
  }
  ekf_.correct(z, H, R, [&](const Eigen::VectorXd & x) { return (H * x).eval(); },
               [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return (a - b).eval(); },
               std::numeric_limits<double>::infinity(), false);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }
const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }
double Target::plate_height(const Eigen::VectorXd & x, int id) const
{
  return x[4] + (id == 0 ? 0.0 : x[8 + id]);
}

Eigen::Vector3d Target::armor_position(const Eigen::VectorXd & x, int id) const
{
  if (id < 0 || id >= armor_num_ || x.size() != 11) {
    throw std::invalid_argument("Invalid target model state/plate id");
  }
  const double angle = x[6] + id * 2.0 * kPi / armor_num_;
  const bool alternate = !is_outpost() && armor_num_ == 4 && id % 2 == 1;
  const double radius = x[8] + (alternate ? x[9] : 0.0);
  return {x[0] - radius * std::cos(angle), x[2] - radius * std::sin(angle),
          is_outpost() ? plate_height(x, id) : x[4] + (alternate ? x[10] : 0.0)};
}

Eigen::Vector4d Target::observation(const Eigen::VectorXd & x, int id) const
{
  const Eigen::Vector3d ypd = tools::xyz2ypd(armor_position(x, id));
  return {ypd[0], ypd[1], ypd[2], tools::limit_rad(x[6] + id * 2.0 * kPi / armor_num_)};
}

Eigen::MatrixXd Target::observation_jacobian(const Eigen::VectorXd & x, int id) const
{
  const Eigen::Vector3d xyz = armor_position(x, id);
  const double angle = x[6] + id * 2.0 * kPi / armor_num_;
  const bool alternate = !is_outpost() && armor_num_ == 4 && id % 2 == 1;
  const double radius = x[8] + (alternate ? x[9] : 0.0);
  Eigen::MatrixXd xyz_jacobian = Eigen::MatrixXd::Zero(3, x.size());
  xyz_jacobian(0, 0) = xyz_jacobian(1, 2) = 1.0;
  xyz_jacobian(0, 6) = radius * std::sin(angle);
  xyz_jacobian(1, 6) = -radius * std::cos(angle);
  xyz_jacobian(0, 8) = -std::cos(angle);
  xyz_jacobian(1, 8) = -std::sin(angle);
  xyz_jacobian(2, 4) = 1.0;
  if (is_outpost() && id != 0) xyz_jacobian(2, 8 + id) = 1.0;
  if (alternate) {
    xyz_jacobian(0, 9) = -std::cos(angle);
    xyz_jacobian(1, 9) = -std::sin(angle);
    xyz_jacobian(2, 10) = 1.0;
  }
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, x.size());
  H.topRows(3) = tools::xyz2ypd_jacobian(xyz) * xyz_jacobian;
  H(3, 6) = 1.0;
  return H;
}

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> result;
  if (!isinit) return result;
  for (int id = 0; id < armor_num_; ++id) {
    const Eigen::Vector3d xyz = armor_position(ekf_.x, id);
    result.emplace_back(xyz[0], xyz[1], xyz[2],
                        tools::limit_rad(ekf_.x[6] + id * 2.0 * kPi / armor_num_));
  }
  return result;
}

bool Target::diverged() const
{
  if (!isinit || ekf_.x.size() != 11 || !ekf_.x.allFinite() ||
      !ekf_.P.allFinite() || (ekf_.P.diagonal().array() < -1e-10).any()) return true;
  const auto & x = ekf_.x;
  if (std::abs(x[0]) > 100.0 || std::abs(x[2]) > 100.0 || std::abs(x[7]) > 20.0 ||
      x[8] < 0.05 || x[8] > 0.5 || ekf_.P(0, 0) > 100.0 || ekf_.P(2, 2) > 100.0) return true;
  if (is_outpost()) {
    for (int id = 0; id < 3; ++id) {
      if (std::abs(plate_height(x, id)) > 20.0) return true;
    }
    if (std::abs(x[5]) > 10.0 || std::abs(x[9]) > 0.6 || std::abs(x[10]) > 0.6) return true;
    return false;
  }
  return armor_num_ == 4 && (x[8] + x[9] < 0.05 || x[8] + x[9] > 0.5);
}

void Target::update_convergency()
{
  is_converged_ = !diverged() && update_count_ >= 5 && ekf_.P(0, 0) < 0.1 &&
    ekf_.P(2, 2) < 0.1 && ekf_.P(4, 4) < 0.01 && ekf_.P(6, 6) < 0.08 &&
    ekf_.P(7, 7) < 1.0 && ekf_.P(8, 8) < 0.0025;
  if (!is_converged_) return;
  const auto & failures = ekf_.recent_nis_failures;
  is_converged_ = failures.size() >= 5 &&
    std::accumulate(failures.begin(), failures.end(), 0) <= 0.2 * failures.size();
  if (is_outpost()) {
    // A stable phase alone does not establish the heights of unseen plates.
    is_converged_ = is_converged_ &&
      *std::min_element(height_observations_.begin(), height_observations_.end()) >= 3 &&
      ekf_.P(9, 9) < 0.0025 && ekf_.P(10, 10) < 0.0025;
  }
}
bool Target::is_converged() const { return is_converged_; }
bool Target::convergened() { update_convergency(); return is_converged_; }
bool Target::checkinit() { return isinit; }
}  // namespace auto_aim
