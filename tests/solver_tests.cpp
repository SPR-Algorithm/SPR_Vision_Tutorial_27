#include "task/auto_aim/solver.hpp"
#include "tools/math_tools.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
constexpr double pi = 3.14159265358979323846;

void require(bool value, const std::string & message)
{
  if (!value) throw std::runtime_error(message);
}

void near(double value, double expected, double tolerance, const std::string & message)
{
  require(std::isfinite(value) && std::abs(value - expected) <= tolerance,
          message + ": got " + std::to_string(value) + ", expected " + std::to_string(expected));
}

// Isolate the mathematical tests from camera files and simulator availability.
struct Configuration
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("auto_aim_solver_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");

  Configuration()
  {
    std::ofstream file(path);
    file << "R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
            "R_camera2gimbal: [0, 0, 1, -1, 0, 0, 0, -1, 0]\n"
            "t_camera2gimbal: [0, 0, 0]\n"
            "camera_matrix: [1000, 0, 640, 0, 1000, 360, 0, 0, 1]\n"
            "distort_coeffs: [0, 0, 0, 0, 0]\n";
    require(bool(file), "write test camera configuration");
  }
  ~Configuration()
  {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
  }
};

auto_aim::Armor armor(const std::vector<cv::Point2f> & corners, auto_aim::ArmorName name,
                      auto_aim::ArmorType type = auto_aim::small)
{
  auto_aim::Armor value(18, 0.99f, cv::Rect{}, corners);
  value.name = name;
  value.type = type;
  return value;
}

void test_world_projection(auto_aim::Solver & solver)
{
  solver.set_world_camera_pose(Eigen::Quaterniond::Identity(), {0, 0, 0});
  const auto pixels = solver.world2pixel({{5, 0, 0}, {5, 1, 0.5}, {-1, 0, 0}, {0, 1, 0}});
  require(pixels.size() == 2, "world2pixel filters zero/negative optical depth");
  near(pixels[0].x, 640, 1e-5, "FLU forward maps to principal point x");
  near(pixels[0].y, 360, 1e-5, "FLU forward maps to principal point y");
  near(pixels[1].x, 440, 1e-5, "FLU left maps to smaller pixel x");
  near(pixels[1].y, 260, 1e-5, "FLU up maps to smaller pixel y");

  const Eigen::Quaterniond q = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d position(2, -1.5, 0.8);
  solver.set_world_camera_pose(q, position);
  std::vector<cv::Point3f> world;
  for (const Eigen::Vector3d & local : {Eigen::Vector3d(5, 0, 0), Eigen::Vector3d(5, 1, 0.5),
                                       Eigen::Vector3d(-1, 0, 0)}) {
    const Eigen::Vector3d p = position + q * local;
    world.emplace_back(p.x(), p.y(), p.z());
  }
  const auto transformed = solver.world2pixel(world);
  require(transformed.size() == 2, "rotated translated camera filters points behind it");
  for (unsigned i = 0; i < 2; ++i) {
    near(transformed[i].x, pixels[i].x, 1e-3, "world pose preserves local projection x");
    near(transformed[i].y, pixels[i].y, 1e-3, "world pose preserves local projection y");
  }
}

void test_pose_roundtrip(auto_aim::Solver & solver)
{
  Eigen::Matrix3d K;
  K << 1150, 0, 711, 0, 1120, 522, 0, 0, 1;
  Eigen::Matrix<double, 5, 1> distortion;
  distortion << -0.03, 0.005, 0.001, -0.002, 0;
  solver.set_camera_calibration(K, distortion);
  for (const double camera_yaw : {0.0, 0.6, -1.2}) {
    const Eigen::Quaterniond q = Eigen::AngleAxisd(camera_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitX());
    const Eigen::Vector3d camera_position(2.0, -1.0, 1.1);
    // A non-unit quaternion must describe the same camera orientation.
    Eigen::Quaterniond scaled_q = q;
    scaled_q.coeffs() *= 2;
    solver.set_world_camera_pose(scaled_q, camera_position);
    for (const auto name : {auto_aim::outpost, auto_aim::sentry}) {
      for (const double relative_yaw : {-0.6, 0.2, 0.65}) {
        const Eigen::Vector3d expected = camera_position + q * Eigen::Vector3d(5.5, 0.2, -0.15);
        const double yaw = tools::limit_rad(camera_yaw + relative_yaw);
        const auto corners = solver.reproject_armor(expected, yaw, auto_aim::small, name);
        auto measured = armor(corners, name);
        require(solver.try_solve(measured), "valid projected armor must solve");
        require((measured.xyz_in_world - expected).norm() < 3e-4,
                "PnP world position roundtrip including translation and distortion");
        const Eigen::Vector3d local = q.conjugate() * (expected - camera_position);
        require((measured.xyz_in_gimbal - local).norm() < 3e-4,
                "PnP camera-link FLU position roundtrip");
        near(tools::limit_rad(measured.ypr_in_world[0] - yaw), 0, 1.1 * pi / 180,
             "optimized world yaw roundtrip");
        require(measured.ypd_in_world.allFinite(), "world spherical pose is finite");
      }
    }
  }
}

void test_invalid_observations(auto_aim::Solver & solver)
{
  solver.set_world_camera_pose(Eigen::Quaterniond::Identity(), {0, 0, 0});
  const auto corners = solver.reproject_armor({5, 0, 1}, 0.2, auto_aim::small, auto_aim::outpost);
  auto malformed = armor(corners, auto_aim::outpost);
  malformed.points.pop_back();
  require(!solver.try_solve(malformed), "reject incomplete corner observations");
  malformed.points = corners;
  malformed.points[0].x = std::numeric_limits<float>::quiet_NaN();
  require(!solver.try_solve(malformed), "reject NaN corners");
  malformed.points[0].x = std::numeric_limits<float>::infinity();
  require(!solver.try_solve(malformed), "reject infinite corners");
  malformed.points = {{1, 1}, {1, 1}, {1, 1}, {1, 1}};
  require(!solver.try_solve(malformed), "reject identical corners");
  malformed.points = {{1, 1}, {2, 2}, {3, 3}, {4, 4}};
  require(!solver.try_solve(malformed), "reject collinear corners despite finite PnP output");
  malformed.points = corners;
  std::swap(malformed.points[1], malformed.points[2]);
  require(!solver.try_solve(malformed), "reject self-intersecting corner order");
}

void test_invalid_camera(auto_aim::Solver & solver)
{
  bool rejected = false;
  try { solver.set_world_camera_pose(Eigen::Quaterniond(0, 0, 0, 0), {0, 0, 0}); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "reject zero quaternion");
  rejected = false;
  try {
    solver.set_world_camera_pose(Eigen::Quaterniond::Identity(),
      {std::numeric_limits<double>::quiet_NaN(), 0, 0});
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "reject non-finite camera position");
  rejected = false;
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = -1000;
  try { solver.set_camera_calibration(K, Eigen::Matrix<double, 5, 1>::Zero()); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "reject negative focal length");
}
}  // namespace

int main()
{
  try {
    Configuration config;
    auto_aim::Solver solver(config.path.string());
    test_world_projection(solver);
    test_pose_roundtrip(solver);
    test_invalid_observations(solver);
    test_invalid_camera(solver);
    std::cout << "Solver coordinate, pose roundtrip, depth, and invalid-input tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Solver test failed: " << error.what() << '\n';
    return 1;
  }
}
