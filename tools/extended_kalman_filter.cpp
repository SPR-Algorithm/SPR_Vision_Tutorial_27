#include "extended_kalman_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, Difference add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.size(), x0.size())), x_add(std::move(add))
{
  if (P.rows() != x.size() || P.cols() != x.size() || !x.allFinite() || !P.allFinite()) {
    throw std::invalid_argument("Invalid EKF initial state/covariance");
  }
  P = (0.5 * (P + P.transpose())).eval();
  for (const auto & key : {"residual_yaw", "residual_pitch", "residual_distance", "residual_angle",
                          "nis", "nees", "nis_fail", "nees_fail", "recent_nis_failures"}) {
    data[key] = 0.0;
  }
  // NEES requires a ground-truth state; a posterior correction is not a NEES sample.
  data["nees_available"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & value) { return (F * value).eval(); });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, Observation f)
{
  if (F.rows() != x.size() || F.cols() != x.size() || Q.rows() != x.size() ||
      Q.cols() != x.size() || !F.allFinite() || !Q.allFinite()) {
    throw std::invalid_argument("Invalid EKF prediction matrices");
  }
  Eigen::VectorXd next_x = f(x);
  Eigen::MatrixXd next_P = F * P * F.transpose() + Q;
  if (!next_x.allFinite() || !next_P.allFinite()) return x;
  x = std::move(next_x);
  P = (0.5 * (next_P + next_P.transpose())).eval();
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  Difference z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & value) { return (H * value).eval(); }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, Observation h,
  Difference z_subtract)
{
  correct(z, H, R, std::move(h), std::move(z_subtract));
  return x;
}

double ExtendedKalmanFilter::nis_threshold(int dimension)
{
  // Upper chi-square quantiles at 99.9%, for a stacked 4D armor observation.
  switch (dimension) {
    case 4: return 18.467;
    case 8: return 26.125;
    case 12: return 32.909;
    case 16: return 39.252;
    default:
      // Wilson-Hilferty approximation; only the 4/8/12/16D cases are used by Target.
      const double n = std::max(1, dimension);
      return n * std::pow(1.0 - 2.0 / (9.0 * n) + 3.090232 * std::sqrt(2.0 / (9.0 * n)), 3);
  }
}

bool ExtendedKalmanFilter::correct(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, Observation h,
  Difference z_subtract, double nis_gate, bool record_stats)
{
  const auto fail = [&](double nis) {
    if (record_stats) record_rejection(nis);
    return false;
  };
  if (z.size() == 0 || H.rows() != z.size() || H.cols() != x.size() || R.rows() != z.size() ||
      R.cols() != z.size() || !z.allFinite() || !H.allFinite() || !R.allFinite()) {
    return fail(std::numeric_limits<double>::infinity());
  }
  const Eigen::VectorXd predicted = h(x);
  if (predicted.size() != z.size() || !predicted.allFinite()) {
    return fail(std::numeric_limits<double>::infinity());
  }
  const Eigen::VectorXd residual = z_subtract(z, predicted);
  const Eigen::MatrixXd PHt = P * H.transpose();
  Eigen::MatrixXd S = H * PHt + R;
  S = (0.5 * (S + S.transpose())).eval();
  Eigen::LDLT<Eigen::MatrixXd> factor(S);
  if (!residual.allFinite() || factor.info() != Eigen::Success ||
      (factor.vectorD().array() <= 0.0).any()) {
    return fail(std::numeric_limits<double>::infinity());
  }
  const double nis = residual.dot(factor.solve(residual));
  if (!std::isfinite(nis) || nis < 0.0 || nis > nis_gate) {
    if (record_stats) record_innovation(residual, nis, false);
    return false;
  }
  const Eigen::MatrixXd K = factor.solve(PHt.transpose()).transpose();
  const Eigen::VectorXd next_x = x_add(x, K * residual);
  const Eigen::MatrixXd complement = I - K * H;
  Eigen::MatrixXd next_P = complement * P * complement.transpose() + K * R * K.transpose();
  next_P = (0.5 * (next_P + next_P.transpose())).eval();
  if (!next_x.allFinite() || !next_P.allFinite() || (next_P.diagonal().array() < 0.0).any()) {
    return fail(nis);
  }
  x = next_x;
  P = std::move(next_P);
  if (record_stats) record_innovation(residual, nis, true);
  return true;
}

void ExtendedKalmanFilter::record_rejection(double nis)
{
  record_innovation(Eigen::VectorXd(), nis, false);
}

void ExtendedKalmanFilter::record_innovation(
  const Eigen::VectorXd & residual, double nis, bool accepted)
{
  last_nis = std::isfinite(nis) ? nis : std::numeric_limits<double>::max();
  last_update_accepted = accepted;
  const bool failed = !accepted || last_nis > nis_threshold(static_cast<int>(residual.size()));
  recent_nis_failures.push_back(failed ? 1 : 0);
  while (recent_nis_failures.size() > std::max<size_t>(1, window_size)) {
    recent_nis_failures.pop_front();
  }
  data["nis"] = last_nis;
  data["nis_fail"] = failed ? 1.0 : 0.0;
  data["update_accepted"] = accepted ? 1.0 : 0.0;
  data["recent_nis_failures"] = static_cast<double>(
    std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0)) /
    recent_nis_failures.size();
  const std::array<const char *, 4> keys{
    "residual_yaw", "residual_pitch", "residual_distance", "residual_angle"};
  for (int i = 0; i < 4; ++i) data[keys[i]] = i < residual.size() ? residual[i] : 0.0;
}
}  // namespace tools
