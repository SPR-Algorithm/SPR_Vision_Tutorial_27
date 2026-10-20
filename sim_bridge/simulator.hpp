#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "talos_layout.hpp"

namespace sim_bridge {
using CameraInfo = talos::CameraInfo;
struct SimFrame {
  cv::Mat bgr;  // Owned BGR8 memory, never a view into simulator mmap storage.
  std::uint64_t sequence = 0, timestamp_ns = 0, received_monotonic_ns = 0;
  std::chrono::steady_clock::time_point timestamp{};
  Eigen::Quaterniond q_world_gimbal = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_gimbal_camera = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_gimbal_muzzle = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_world_gimbal = Eigen::Vector3d::Zero();
  Eigen::Vector3d t_gimbal_camera = Eigen::Vector3d::Zero();
  Eigen::Vector3d t_gimbal_muzzle = Eigen::Vector3d::Zero();
  CameraInfo camera{};
  bool following = false, discontinuity = false;
  // Optional companion counters in RuntimeState reserved bytes; monitoring
  // only, not synchronized ground truth or part of the firing decision.
  std::uint32_t launch_count = 0, hit_count = 0;
  std::uint64_t generation = 0;
};
struct Stats {
  std::uint64_t received = 0, replaced = 0, mismatch = 0, reconnects = 0;
  std::uint64_t stale_heartbeat = 0, clock_resets = 0, malformed = 0;
};
// One Simulator is the sole image/pose consumer and command producer for an IPC
// directory. read() is latest-only; frames retain ownership across later reads.
class Simulator {
public:
  explicit Simulator(
    std::string directory = "/tmp",
    std::chrono::milliseconds stale_after = std::chrono::milliseconds(1500));
  ~Simulator();
  Simulator(const Simulator &) = delete;
  Simulator & operator=(const Simulator &) = delete;
  bool read(SimFrame & frame, std::chrono::milliseconds timeout);
  void send(double yaw_rad, double pitch_rad, bool fire, double distance);
  Stats stats() const;
  bool connected() const;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sim_bridge
