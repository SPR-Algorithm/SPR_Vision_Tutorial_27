#include "task/auto_aim/target.hpp"
#include "task/auto_aim/tracker.hpp"
#include "tools/math_tools.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <string>

namespace
{
constexpr double pi = 3.14159265358979323846;
const auto epoch = std::chrono::steady_clock::time_point{};

void require(bool value, const std::string & message)
{
  if (!value) throw std::runtime_error(message);
}
void near(double value, double expected, double tolerance, const std::string & message)
{
  require(std::abs(value - expected) <= tolerance,
          message + ": got " + std::to_string(value) + ", expected " + std::to_string(expected));
}

auto_aim::Armor armor(
  int id, double theta = 0.2, auto_aim::ArmorName name = auto_aim::ArmorName::outpost,
  int count = 3, double radius = 0.2765)
{
  const std::vector<cv::Point2f> corners{{0, 0}, {30, 0}, {30, 12}, {0, 12}};
  auto_aim::Armor value(18, 0.99f, cv::Rect(0, 0, 30, 12), corners);
  value.name = name;
  value.type = auto_aim::ArmorType::small;
  value.color = auto_aim::Color::red;
  value.priority = auto_aim::ArmorPriority::first;
  const double angle = tools::limit_rad(theta + id * 2.0 * pi / count);
  const double height = name == auto_aim::ArmorName::outpost ? 1.2 - 0.102 * id : 0.8;
  value.xyz_in_world = {5.0 - radius * std::cos(angle), 0.4 - radius * std::sin(angle), height};
  value.ypr_in_world = {angle, -15.0 * pi / 180.0, 0.0};
  value.ypd_in_world = tools::xyz2ypd(value.xyz_in_world);
  return value;
}

void test_initial_covariance()
{
  for (int initial = 0; initial < 3; ++initial) {
    auto_aim::Target target(armor(initial), epoch, 0.2765, 3);
    require(target.ekf_x().size() == 11, "outpost dimension");
    require(target.armor_num() == 3, "outpost plate count");
    require((target.ekf().P.diagonal().array() > 0.0).all(), "positive default covariance");
    near(target.ekf_x()[4], 1.2 - 0.102 * initial, 1e-12, "reference is first observed height");
    near(target.ekf_x()[9], 0.0, 1e-12, "unknown dh1 starts uncommitted");
    near(target.ekf_x()[10], 0.0, 1e-12, "unknown dh2 starts uncommitted");
  }
  Eigen::VectorXd custom = Eigen::VectorXd::LinSpaced(11, 0.01, 0.11);
  auto_aim::Target configured(armor(0), epoch, 0.2765, 3, custom);
  require((configured.ekf().P.diagonal() - custom).norm() < 1e-12, "caller P0 actually applied");
  custom.tail<3>().setZero();
  auto_aim::Target sanitized(armor(0), epoch, 0.2765, 3, custom);
  require((sanitized.ekf().P.diagonal().array() > 0.0).all(), "zero geometry variances repaired");
  bool rejected = false;
  try { auto_aim::Target wrong(armor(0), epoch, 0.2765, 3, Eigen::VectorXd::Ones(13)); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "incompatible 13D covariance rejected explicitly");
}

void test_jacobians()
{
  for (const int count : {2, 3, 4}) {
    const auto name = count == 3 ? auto_aim::ArmorName::outpost : auto_aim::ArmorName::three;
    auto_aim::Target target(armor(0, 0.2, name, count, 0.2), epoch, 0.2, count);
    Eigen::VectorXd x = target.ekf_x();
    x[0] = 4.3; x[2] = -0.7; x[4] = 0.6; x[6] = 0.35; x[8] = 0.26;
    x[9] = count == 3 ? -0.102 : 0.035;
    x[10] = count == 3 ? 0.102 : 0.08;
    for (int id = 0; id < count; ++id) {
      const Eigen::MatrixXd H = target.observation_jacobian(x, id);
      Eigen::MatrixXd numeric(4, 11);
      for (int k = 0; k < 11; ++k) {
        auto a = x, b = x;
        a[k] += 1e-6; b[k] -= 1e-6;
        Eigen::Vector4d difference = target.observation(a, id) - target.observation(b, id);
        difference[0] = tools::limit_rad(difference[0]);
        difference[3] = tools::limit_rad(difference[3]);
        numeric.col(k) = difference / 2e-6;
      }
      require((H - numeric).cwiseAbs().maxCoeff() < 2e-7,
              "finite difference h/H count=" + std::to_string(count) + " id=" + std::to_string(id));
    }
    if (count == 3) {
      near(target.armor_position(x, 1)[2], x[4] + x[9], 1e-12, "outpost dh1 height");
      near(target.armor_position(x, 2)[2], x[4] + x[10], 1e-12, "outpost dh2 height");
      auto shifted = x; shifted[9] += 0.03;
      require((target.armor_position(shifted, 1).head<2>() -
               target.armor_position(x, 1).head<2>()).norm() < 1e-12, "outpost dh never changes radius");
    } else if (count == 4) {
      near((target.armor_position(x, 1).head<2>() - Eigen::Vector2d(x[0], x[2])).norm(),
           x[8] + x[9], 1e-12, "vehicle long radius");
      near(target.armor_position(x, 1)[2], x[4] + x[10], 1e-12, "vehicle alternate height");
    }
  }
}

void test_prior_nis_and_rejection()
{
  Eigen::VectorXd x(1); x << 0.0;
  Eigen::MatrixXd P(1, 1); P << 1.0;
  tools::ExtendedKalmanFilter filter(x, P);
  Eigen::VectorXd z(1); z << 2.0;
  Eigen::MatrixXd H(1, 1); H << 1.0;
  Eigen::MatrixXd R(1, 1); R << 1.0;
  auto h = [](const Eigen::VectorXd & value) { return value; };
  auto subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return (a - b).eval(); };
  require(!filter.correct(z, H, R, h, subtract, 1.0), "prior NIS gate rejects");
  near(filter.last_nis, 2.0, 1e-12, "NIS uses prior innovation covariance");
  near(filter.x[0], 0.0, 1e-12, "rejected state unchanged");
  near(filter.P(0, 0), 1.0, 1e-12, "rejected covariance unchanged");
  require(filter.correct(z, H, R, h, subtract, 3.0), "valid update accepted");
  near(filter.x[0], 1.0, 1e-12, "Kalman mean");
  near(filter.P(0, 0), 0.5, 1e-12, "Joseph covariance");
  near(filter.last_nis, 2.0, 1e-12, "accepted NIS still prior");
  require(filter.data.at("nees_available") == 0.0, "no fake NEES without truth");
}

void test_dt_covariance()
{
  auto_aim::Target one(armor(0), epoch, 0.2765, 3), many = one;
  one.predict(0.1);
  for (int i = 0; i < 10; ++i) many.predict(0.01);
  require((one.ekf().P - many.ekf().P).norm() < 1e-10, "Q has continuous-time dt scaling");
  const auto prior = one.ekf_x();
  one.predict(-0.1);
  one.predict(std::numeric_limits<double>::quiet_NaN());
  require((one.ekf_x() - prior).norm() == 0.0, "invalid dt leaves state unchanged");
}

void test_batch_and_two_plates()
{
  auto_aim::Target forward(armor(0), epoch, 0.2765, 3), reversed = forward;
  forward.predict(0.01); reversed.predict(0.01);
  std::vector<auto_aim::Armor> observations{armor(0), armor(1)};
  require(forward.update(observations) == 2, "two observations accepted jointly");
  std::reverse(observations.begin(), observations.end());
  require(reversed.update(observations) == 2, "input order does not change acceptance");
  require((forward.ekf_x() - reversed.ekf_x()).norm() < 1e-9, "joint update is order independent");
  std::set<int> ids;
  for (const auto [index, id] : forward.last_associations()) ids.insert(id);
  require(ids.size() == 2, "batch matches physical plates one to one");
  auto_aim::Target duplicates(armor(0), epoch, 0.2765, 3);
  require(duplicates.update(std::vector<auto_aim::Armor>{armor(0), armor(0)}) == 1,
          "duplicate detections cannot update the same plate twice");
  auto_aim::Target balance(armor(0, 0.2, auto_aim::ArmorName::three, 2, 0.2), epoch, 0.2, 2);
  balance.predict(0.01);
  require(balance.update(std::vector<auto_aim::Armor>{
    armor(0, 0.2, auto_aim::ArmorName::three, 2, 0.2),
    armor(1, 0.2, auto_aim::ArmorName::three, 2, 0.2)}) == 2, "2-plate target never reads a third plate");
  require(balance.ekf_x().size() == 11 && !balance.diverged(), "vehicle remains valid 11D");
}

void test_joint_gate_rejects_inconsistent_frame()
{
  auto_aim::Target baseline(armor(0), epoch, 0.2765, 3);
  auto first = armor(0), second = armor(1);
  first.xyz_in_world[0] += 0.8;
  second.xyz_in_world[0] -= 0.8;
  first.ypd_in_world = tools::xyz2ypd(first.xyz_in_world);
  second.ypd_in_world = tools::xyz2ypd(second.xyz_in_world);
  auto independent_first = baseline, independent_second = baseline;
  require(independent_first.update(std::vector<auto_aim::Armor>{first}) == 1,
          "first candidate passes its individual gate");
  require(independent_second.update(std::vector<auto_aim::Armor>{second}) == 1,
          "second candidate passes its individual gate");
  require(baseline.update(std::vector<auto_aim::Armor>{first, second}) == 1,
          "joint gate prevents mutually incompatible center corrections");
}

void test_invalid_measurements()
{
  auto_aim::Target target(armor(0), epoch, 0.2765, 3);
  const auto prior = target.ekf_x();
  const auto covariance = target.ekf().P;
  auto malformed = armor(0);
  malformed.xyz_in_world[0] = std::numeric_limits<double>::quiet_NaN();
  require(target.update(std::vector<auto_aim::Armor>{malformed}) == 0, "NaN observation rejected");
  auto outlier = armor(0);
  outlier.xyz_in_world[0] += 10.0;
  outlier.ypd_in_world = tools::xyz2ypd(outlier.xyz_in_world);
  require(target.update(std::vector<auto_aim::Armor>{outlier}) == 0, "gross position outlier rejected");
  require((target.ekf_x() - prior).norm() == 0.0, "bad observation leaves state unchanged");
  require((target.ekf().P - covariance).norm() == 0.0, "bad observation leaves covariance unchanged");
  require(target.update_count() == 0, "rejection does not increase update count");
}

void test_measurement_stddev_scale()
{
  const auto initial = armor(0);
  const auto covariance = Eigen::VectorXd::Constant(11, 1e-8);
  auto_aim::Target baseline(initial, epoch, 0.2765, 3, covariance);
  auto scaled = baseline;
  scaled.set_measurement_stddev_scale(Eigen::Vector4d::Constant(2.0));
  auto jitter = initial;
  jitter.xyz_in_world.y() += 0.08;
  jitter.ypd_in_world = tools::xyz2ypd(jitter.xyz_in_world);
  require(baseline.update(std::vector<auto_aim::Armor>{jitter}) == 0,
          "underestimated nominal noise rejects 8 cm bearing jitter");
  require(scaled.update(std::vector<auto_aim::Armor>{jitter}) == 1,
          "configured doubled standard deviation accepts plausible PnP jitter");
  require(scaled.ekf().last_nis < baseline.ekf().last_nis,
          "standard deviation multiplier changes prior NIS through covariance");
  near(scaled.ekf().last_nis, baseline.ekf().last_nis / 4.0, 0.03,
       "sigma doubled means R quadrupled, not doubled");
  auto outlier = initial;
  outlier.xyz_in_world.x() += 10.0;
  outlier.ypd_in_world = tools::xyz2ypd(outlier.xyz_in_world);
  require(scaled.update(std::vector<auto_aim::Armor>{outlier}) == 0,
          "scaled measurement covariance does not admit gross position outliers");

  auto_aim::Target horizontal(initial, epoch, 0.2765, 3, covariance);
  auto vertical = horizontal;
  horizontal.set_measurement_stddev_scale({2.0, 1.0, 1.0, 1.0});
  vertical.set_measurement_stddev_scale({1.0, 2.0, 1.0, 1.0});
  require(horizontal.update(std::vector<auto_aim::Armor>{jitter}) == 1,
          "first vector component scales bearing yaw");
  require(vertical.update(std::vector<auto_aim::Armor>{jitter}) == 0,
          "bearing pitch multiplier does not relax bearing yaw");

  auto_aim::Target unchanged(initial, epoch, 0.2765, 3, covariance);
  auto explicit_default = unchanged;
  explicit_default.set_measurement_stddev_scale(Eigen::Vector4d::Ones());
  require(unchanged.update(std::vector<auto_aim::Armor>{initial}) == 1 &&
          explicit_default.update(std::vector<auto_aim::Armor>{initial}) == 1,
          "default noise remains accepted");
  require((unchanged.ekf().P - explicit_default.ekf().P).norm() == 0.0,
          "default ones preserve existing covariance exactly");
  for (double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    auto value = Eigen::Vector4d::Ones().eval(); value[2] = invalid;
    bool rejected = false;
    try { scaled.set_measurement_stddev_scale(value); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "noise setter rejects nonpositive/nonfinite values");
  }
}

void test_tracker_measurement_noise_configuration()
{
  struct Configuration {
    std::filesystem::path path;
    explicit Configuration(const std::string & settings) {
      static int sequence = 0;
      path = std::filesystem::temp_directory_path() / ("target_noise_test_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
        "_" + std::to_string(sequence++) + ".yaml");
      std::ofstream file(path);
      file << "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
              "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
              "t_camera2gimbal: [0,0,0]\n"
              "camera_matrix: [1000,0,640,0,1000,360,0,0,1]\n"
              "distort_coeffs: [0,0,0,0,0]\n" << settings;
      require(bool(file), "write noise configuration fixture");
    }
    ~Configuration() { std::error_code ignored; std::filesystem::remove(path, ignored); }
  };
  Configuration defaults("");
  auto_aim::Solver solver(defaults.path.string());
  auto_aim::Tracker nominal(defaults.path.string(), solver);
  Configuration scalar("target: {measurement_stddev_scale: 2.0}\n");
  Configuration vector("target: {measurement_stddev_scale: [2, 2, 2, 2]}\n");
  auto_aim::Tracker doubled(scalar.path.string(), solver), componentwise(vector.path.string(), solver);
  auto observed = armor(0);
  observed.points = solver.reproject_armor(observed.xyz_in_world, observed.ypr_in_world[0],
                                           observed.type, observed.name);
  std::list<auto_aim::Target> a, b, c;
  for (int frame = 0; frame < 2; ++frame) {
    std::list<auto_aim::Armor> first{observed}, second{observed}, third{observed};
    const auto time = epoch + std::chrono::milliseconds(frame * 10);
    a = nominal.track(first, time); b = doubled.track(second, time); c = componentwise.track(third, time);
    require(a.size() == 1 && b.size() == 1 && c.size() == 1, "configured trackers acquire PnP target");
  }
  require(b.front().ekf().P(0, 0) > a.front().ekf().P(0, 0),
          "Tracker applies configured scale after target acquisition");
  require((b.front().ekf().P - c.front().ekf().P).norm() < 1e-12,
          "scalar and four equal multipliers have identical meaning");
  for (const std::string settings : {"0", "-1", ".nan", ".inf", "[1, 2]",
                                     "[1, 1, 0, 1]", "[1, .nan, 1, 1]", "{invalid: 2}"}) {
    Configuration invalid("target: {measurement_stddev_scale: " + settings + "}\n");
    bool rejected = false;
    try { auto_aim::Tracker bad(invalid.path.string(), solver); }
    catch (const std::exception &) { rejected = true; }
    require(rejected, "invalid YAML noise scale is rejected before tracking");
  }
}

void test_rotation_and_initial_height()
{
  for (const double speed : {-2.3, 2.3}) {
    for (int first = 0; first < 3; ++first) {
      auto_aim::Target target(armor(first), epoch, 0.2765, 3);
      std::mt19937 random(42 + first);
      std::normal_distribution<double> position_noise(0.0, 0.002);
      std::normal_distribution<double> yaw_noise(0.0, 0.01);
      double theta = 0.2;
      int accepted = 0;
      for (int frame = 0; frame < 650; ++frame) {
        theta += speed * 0.01;
        target.predict(0.01);
        // Track one visible plate at a time. The first reference can have any height.
        int visible = 0;
        for (int id = 1; id < 3; ++id) {
          if (armor(id, theta).xyz_in_world.norm() < armor(visible, theta).xyz_in_world.norm()) visible = id;
        }
        auto measured = armor(visible, theta);
        for (int k = 0; k < 3; ++k) measured.xyz_in_world[k] += position_noise(random);
        measured.ypr_in_world[0] += yaw_noise(random);
        measured.ypd_in_world = tools::xyz2ypd(measured.xyz_in_world);
        accepted += target.update(std::vector<auto_aim::Armor>{measured});
      }
      const auto x = target.ekf_x();
      require(accepted > 600, "single-plate rotation updates remain associated");
      near(x[7], speed, 0.18, "both rotation directions estimated");
      near(x[0], 5.0, 0.06, "center x converges");
      near(x[2], 0.4, 0.06, "center y converges");
      near(x[8], 0.2765, 0.025, "radius converges");
      for (int id = 0; id < 3; ++id) {
        near(target.armor_position(x, id)[2], 1.2 - 0.102 * ((first + id) % 3), 0.04,
             "height identity relative to arbitrary first plate");
      }
      require(!target.diverged() && target.is_converged(), "steady rotation converges");
      require((target.ekf().P - target.ekf().P.transpose()).norm() < 1e-10, "covariance remains symmetric");
      Eigen::LDLT<Eigen::MatrixXd> factor(target.ekf().P);
      require((factor.vectorD().array() > 0.0).all(), "covariance remains positive definite");
    }
  }
}

void test_stop_and_reverse()
{
  auto_aim::Target target(armor(0), epoch, 0.2765, 3);
  double theta = 0.2;
  for (int stage = 0; stage < 3; ++stage) {
    const double speed = stage == 0 ? 2.3 : (stage == 1 ? 0.0 : -2.3);
    for (int frame = 0; frame < 250; ++frame) {
      theta += speed * 0.01;
      target.predict(0.01);
      std::vector<auto_aim::Armor> measurements;
      for (int id = 0; id < 3; ++id) measurements.push_back(armor(id, theta));
      require(target.update(measurements) > 0, "stop/reverse does not lock old rotation sign");
    }
    near(target.ekf_x()[7], speed, 0.12, "speed follows rotate/stop/reverse");
  }
}

void test_convergence_requires_observable_heights_and_valid_innovations()
{
  auto_aim::Target single(armor(0), epoch, 0.2765, 3);
  for (int frame = 0; frame < 300; ++frame) {
    single.predict(0.01);
    require(single.update(std::vector<auto_aim::Armor>{armor(0)}) == 1, "single plate stays tracked");
  }
  require(!single.is_converged(), "unobserved outpost heights cannot be declared converged");

  auto_aim::Target complete(armor(0), epoch, 0.2765, 3);
  const std::vector<auto_aim::Armor> observations{armor(0), armor(1), armor(2)};
  for (int frame = 0; frame < 300; ++frame) {
    complete.predict(0.01);
    require(complete.update(observations) == 3, "all plates observed");
  }
  require(complete.is_converged(), "observable stationary outpost converges");
  auto outlier = armor(0);
  outlier.xyz_in_world[0] += 10.0;
  outlier.ypd_in_world = tools::xyz2ypd(outlier.xyz_in_world);
  for (int frame = 0; frame < 25; ++frame) complete.update(outlier);
  require(!complete.is_converged(), "repeated innovation failures invalidate convergence");
  for (int frame = 0; frame < 100; ++frame) {
    complete.predict(0.01);
    require(complete.update(observations) == 3, "valid measurements recover after outliers");
  }
  require(complete.is_converged(), "valid innovation history restores convergence");
  complete.predict(1.0);
  require(!complete.is_converged(), "long observation loss grows uncertainty and removes convergence");
}
}  // namespace

int main()
{
  try {
    test_initial_covariance();
    test_jacobians();
    test_prior_nis_and_rejection();
    test_dt_covariance();
    test_batch_and_two_plates();
    test_joint_gate_rejects_inconsistent_frame();
    test_invalid_measurements();
    test_measurement_stddev_scale();
    test_tracker_measurement_noise_configuration();
    test_rotation_and_initial_height();
    test_stop_and_reverse();
    test_convergence_requires_observable_heights_and_valid_innovations();
    std::cout << "target_tests: all checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "target_tests: " << error.what() << '\n';
    return 1;
  }
}
