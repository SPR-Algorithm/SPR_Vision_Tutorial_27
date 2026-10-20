#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <string>

namespace tools
{
class ExtendedKalmanFilter
{
public:
  using Difference =
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)>;
  using Observation = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;

  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  ExtendedKalmanFilter() = default;
  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    Difference x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);
  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, Observation f);
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    Difference z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, Observation h,
    Difference z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  // Gate the PRIOR innovation. Rejected observations leave x and P unchanged.
  // Set record_stats=false for physical soft constraints, which are not sensor observations.
  bool correct(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, Observation h,
    Difference z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
    double nis_gate = std::numeric_limits<double>::infinity(), bool record_stats = true);
  void record_rejection(double nis = std::numeric_limits<double>::infinity());
  static double nis_threshold(int observation_dimension);

  std::map<std::string, double> data;
  std::deque<int> recent_nis_failures;
  size_t window_size = 100;
  double last_nis = 0.0;
  bool last_update_accepted = false;

private:
  Eigen::MatrixXd I;
  Difference x_add;
  void record_innovation(const Eigen::VectorXd & residual, double nis, bool accepted);
};
}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP
