#ifndef SIM_BRIDGE__FOXGLOVE_DEBUG_HPP
#define SIM_BRIDGE__FOXGLOVE_DEBUG_HPP

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sim_bridge
{

struct DebugFrame
{
  uint64_t timestamp_ns = 0;
  cv::Mat image;
  nlohmann::json telemetry = nlohmann::json::object();
  std::vector<Eigen::Vector3d> measured, predicted;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  bool has_target = false;
};

// Move an owned image into publish(); pixels must not be mutated after publication.
// A single pending frame replaces older input. JPEG/JSON encoding and socket I/O
// run off the inference thread; each client also has bounded latest-only output.
class FoxgloveDebug
{
public:
  // Metrics use rate_hz; image and scene use visual_rate_hz (0 inherits rate_hz).
  explicit FoxgloveDebug(
    std::string host = "127.0.0.1", uint16_t port = 8765, double rate_hz = 10.0,
    double visual_rate_hz = 0.0);
  ~FoxgloveDebug();
  FoxgloveDebug(const FoxgloveDebug &) = delete;
  FoxgloveDebug & operator=(const FoxgloveDebug &) = delete;

  void publish(DebugFrame frame);
  uint16_t port() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sim_bridge

#endif
