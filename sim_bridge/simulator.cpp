#include "simulator.hpp"

#include <opencv2/imgproc.hpp>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace sim_bridge {
namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
std::uint64_t unix_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
std::uint8_t load_state(const std::uint8_t & state) {
  return __atomic_load_n(&state, __ATOMIC_ACQUIRE);
}
template <typename T> bool take(talos::TripleBuffer<T> & buffer, T & result) {
  auto expected = load_state(buffer.state);
  for (int attempt = 0; attempt < 4; ++attempt) {
    if (!(expected & talos::kNew)) return false;
    const auto ready = expected & talos::kIndexMask;
    if (ready > 2 || buffer.read_idx > 2) return false;
    const auto desired = buffer.read_idx;
    if (__atomic_compare_exchange_n(
          &buffer.state, &expected, desired, true, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
      buffer.read_idx = ready;
      result = buffer.slots[ready];
      return true;
    }
  }
  return false;
}
template <typename T> bool publish(talos::TripleBuffer<T> & buffer, const T & value) {
  if (buffer.write_idx > 2 || (load_state(buffer.state) & talos::kIndexMask) > 2) return false;
  buffer.slots[buffer.write_idx] = value;
  const auto previous = __atomic_exchange_n(
    &buffer.state, std::uint8_t(buffer.write_idx | talos::kNew), __ATOMIC_ACQ_REL);
  buffer.write_idx = previous & talos::kIndexMask;
  return true;
}
struct Mapping {
  int fd = -1;
  void * data = MAP_FAILED;
  std::size_t length = 0;
  dev_t device{};
  ino_t inode{};
  ~Mapping() { close(); }
  void close() {
    if (data != MAP_FAILED) ::munmap(data, length);
    if (fd >= 0) ::close(fd);
    fd = -1; data = MAP_FAILED; length = 0;
  }
  bool open(const std::string & path, std::size_t bytes) {
    close();
    fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    struct stat st{};
    if (fd < 0 || ::fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(bytes)) {
      close(); return false;
    }
    data = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) { close(); return false; }
    length = bytes; device = st.st_dev; inode = st.st_ino;
    return true;
  }
  bool current(const std::string & path) const {
    struct stat st{};
    return fd >= 0 && ::stat(path.c_str(), &st) == 0 && st.st_dev == device &&
           st.st_ino == inode && st.st_size >= static_cast<off_t>(length);
  }
};
Eigen::Vector3d vector(const float (&v)[3]) { return {v[0], v[1], v[2]}; }
bool finite_camera(const CameraInfo & c) {
  if (c.width != talos::kWidth || c.height != talos::kHeight ||
      !std::isfinite(c.fx) || !std::isfinite(c.fy) || c.fx <= 0 || c.fy <= 0 ||
      !std::isfinite(c.cx) || !std::isfinite(c.cy)) return false;
  for (auto v : c.distortion) if (!std::isfinite(v)) return false;
  return true;
}
}  // namespace

class Simulator::Impl {
public:
  Impl(std::string directory, std::chrono::milliseconds stale_after)
  : meta_path_(directory + "/" + talos::kMetaName),
    pool_path_(directory + "/" + talos::kPoolName),
    stale_after_(std::max(stale_after, 10ms)), thread_([this] { run(); }) {}
  ~Impl() {
    stop_.store(true);
    frame_ready_.notify_all(); wake_.notify_all();
    thread_.join();
  }
  bool read(SimFrame & frame, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    frame_ready_.wait_for(lock, timeout, [&] { return stop_.load() || latest_.has_value(); });
    if (!latest_) return false;
    // Recycle only uniquely owned caller storage. A retained cv::Mat copy must
    // never become a writable receive buffer.
    recycle(frame.bgr);
    frame = std::move(*latest_);
    latest_.reset();
    return true;
  }
  void send(double yaw, double pitch, bool fire, double distance) {
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(distance)) return;
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!connected_.load() || !meta_) return;
    constexpr double degrees = 180.0 / 3.14159265358979323846;
    talos::GimbalCmd command{};
    command.timestamp_ns = unix_ns();
    command.yaw_deg = static_cast<float>(yaw * degrees);
    command.pitch_deg = static_cast<float>(-pitch * degrees);
    command.distance_m = static_cast<float>(distance);
    command.fire_advice = fire ? 1 : 0;
    if (!std::isfinite(command.yaw_deg) || !std::isfinite(command.pitch_deg) ||
        !std::isfinite(command.distance_m)) return;
    // Pose updates are latest-only. Retain the most recent launch token so a
    // following fire=false pose cannot erase a pulse before Bevy consumes it.
    // Bevy deduplicates tokens and expires the ORIGINAL request after 100 ms.
    if (fire) last_fire_token_ = command.timestamp_ns;
    for (int byte = 0; byte < 8; ++byte)
      command.pad[byte] = static_cast<std::uint8_t>(last_fire_token_ >> (byte * 8));
    publish(meta_->gimbal_cmd, command);
  }
  Stats stats() const {
    return {received_.load(), replaced_.load(), mismatch_.load(), reconnects_.load(),
            stale_.load(), clock_resets_.load(), malformed_.load()};
  }
  bool connected() const { return connected_.load(); }

private:
  void recycle(cv::Mat & image) {
    if (!image.empty() && image.u && image.u->refcount == 1 && spare_.size() < 2)
      spare_.push_back(std::move(image));
  }
  cv::Mat buffer() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!spare_.empty()) {
      auto image = std::move(spare_.back()); spare_.pop_back(); return image;
    }
    return cv::Mat(talos::kHeight, talos::kWidth, CV_8UC3);
  }
  void disconnect() {
    connected_.store(false);
    meta_ = nullptr;
    meta_mapping_.close(); pool_mapping_.close();
    have_previous_ = false; pending_discontinuity_ = true;
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_) { recycle(latest_->bgr); latest_.reset(); }
  }
  bool connect() {
    if (!meta_mapping_.open(meta_path_, sizeof(talos::ShmMetaRegion)) ||
        !pool_mapping_.open(pool_path_, talos::kPoolSize)) {
      disconnect(); return false;
    }
    meta_ = static_cast<talos::ShmMetaRegion *>(meta_mapping_.data);
    if (meta_->header.magic != talos::kMagic || meta_->header.version != talos::kVersion ||
        meta_->header.image_width != talos::kWidth || meta_->header.image_height != talos::kHeight ||
        meta_->header.created_ns == 0) {
      disconnect(); return false;
    }
    created_ = meta_->header.created_ns;
    heartbeat_ = __atomic_load_n(&meta_->header.heartbeat_ns, __ATOMIC_ACQUIRE);
    if (created_ == stale_created_ && heartbeat_ == stale_heartbeat_) {
      disconnect(); return false;
    }
    heartbeat_changed_ = Clock::now();
    ++generation_; ++reconnects_;
    pending_discontinuity_ = true; have_previous_ = false;
    connected_.store(true);
    return true;
  }
  bool healthy() {
    if (!meta_mapping_.current(meta_path_) || !pool_mapping_.current(pool_path_)) return false;
    if (meta_->header.magic != talos::kMagic || meta_->header.version != talos::kVersion ||
        meta_->header.created_ns != created_) return false;
    const auto heartbeat = __atomic_load_n(&meta_->header.heartbeat_ns, __ATOMIC_ACQUIRE);
    const auto now = Clock::now();
    if (heartbeat != heartbeat_) { heartbeat_ = heartbeat; heartbeat_changed_ = now; }
    if (now - heartbeat_changed_ > stale_after_) {
      stale_created_ = created_; stale_heartbeat_ = heartbeat_; ++stale_; return false;
    }
    return true;
  }
  void map_time(SimFrame & frame, Clock::time_point now) {
    const auto source = frame.timestamp_ns;
    bool reset = pending_discontinuity_;
    if (have_previous_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - previous_received_).count();
      if (frame.sequence <= previous_sequence_ || source <= previous_source_ ||
          source - previous_source_ > std::uint64_t(std::max<std::int64_t>(elapsed, 0)) + 1000000000ULL) {
        reset = true; ++generation_; ++clock_resets_;
      }
    }
    if (!have_previous_ || reset) {
      source_anchor_ = source;
      steady_anchor_ = now;
      const auto wall = unix_ns();
      // Epoch and steady clocks have different origins. Preserve a plausible
      // capture age; a clock jump/restart still resets the tracker explicitly.
      if (wall >= source && wall - source < 10000000000ULL)
        steady_anchor_ -= std::chrono::nanoseconds(wall - source);
    }
    frame.timestamp = steady_anchor_ + std::chrono::nanoseconds(source - source_anchor_);
    if (have_mapped_ && frame.timestamp <= previous_mapped_) {
      // A discontinuity must not feed negative dt even when a caller misses its flag.
      frame.timestamp = std::max(now, previous_mapped_ + 1ns);
      steady_anchor_ = frame.timestamp; source_anchor_ = source;
      if (!reset) { ++generation_; ++clock_resets_; reset = true; }
    }
    frame.discontinuity = reset; frame.generation = generation_;
    previous_source_ = source; previous_sequence_ = frame.sequence;
    previous_received_ = now; previous_mapped_ = frame.timestamp;
    have_previous_ = true; have_mapped_ = true; pending_discontinuity_ = false;
  }
  bool receive() {
    if (!(load_state(meta_->image.state) & talos::kNew)) return false;
    for (int i = 0; i < 4; ++i)
      if (!(load_state(meta_->poses[i].state) & talos::kNew)) return false;
    talos::ImageMeta image{};
    if (!take(meta_->image, image)) { ++malformed_; return false; }
    const bool valid_image = image.width == talos::kWidth && image.height == talos::kHeight &&
                             image.buffer_id < 3 && image.format == 0 && image.timestamp_ns > 0;
    SimFrame frame;
    if (valid_image) {
      frame.bgr = buffer();
      const auto * pixels = static_cast<std::uint8_t *>(pool_mapping_.data) + image.buffer_id * talos::kImageSize;
      const cv::Mat rgb(talos::kHeight, talos::kWidth, CV_8UC3, const_cast<std::uint8_t *>(pixels));
      // All four pose NEW bits are still set: the Rust producer cannot publish
      // again until this pixel copy has finished and we consume the final pose.
      cv::cvtColor(rgb, frame.bgr, cv::COLOR_RGB2BGR);
    }
    talos::PoseMeta poses[4]{};
    bool valid_poses = true;
    for (int i = 0; i < 4; ++i) {
      if (!take(meta_->poses[i], poses[i])) valid_poses = false;
      if (poses[i].frame_seq != image.seq || poses[i].timestamp_ns != image.timestamp_ns)
        valid_poses = false;
    }
    if (!valid_image) { ++malformed_; return true; }
    if (!valid_poses) { ++mismatch_; return true; }
    frame.camera = meta_->camera_info;
    frame.q_world_gimbal = Eigen::Quaterniond(
      poses[0].quaternion[0], poses[0].quaternion[1], poses[0].quaternion[2], poses[0].quaternion[3]);
    frame.q_gimbal_muzzle = Eigen::Quaterniond(
      poses[2].quaternion[0], poses[2].quaternion[1], poses[2].quaternion[2], poses[2].quaternion[3]);
    frame.q_gimbal_camera = Eigen::Quaterniond(
      poses[3].quaternion[0], poses[3].quaternion[1], poses[3].quaternion[2], poses[3].quaternion[3]);
    frame.p_world_gimbal = vector(poses[1].position);
    frame.t_gimbal_muzzle = vector(poses[2].position);
    frame.t_gimbal_camera = vector(poses[3].position);
    if (!finite_camera(frame.camera) || !frame.q_world_gimbal.coeffs().allFinite() ||
        frame.q_world_gimbal.norm() < 1e-6 || !frame.q_gimbal_camera.coeffs().allFinite() ||
        !frame.q_gimbal_muzzle.coeffs().allFinite() || frame.q_gimbal_camera.norm() < 1e-6 ||
        frame.q_gimbal_muzzle.norm() < 1e-6 || !frame.p_world_gimbal.allFinite() ||
        !frame.t_gimbal_camera.allFinite() || !frame.t_gimbal_muzzle.allFinite()) {
      ++malformed_; return true;
    }
    frame.q_world_gimbal.normalize();
    frame.q_gimbal_camera.normalize(); frame.q_gimbal_muzzle.normalize();
    frame.following = __atomic_load_n(&meta_->runtime_state.following, __ATOMIC_ACQUIRE) != 0;
    const auto counter = [&](std::size_t offset) {
      // Companion publishes each naturally aligned little-endian counter
      // atomically. They are independent diagnostics, not a paired snapshot.
      const auto *value = reinterpret_cast<const std::uint32_t *>(
        &meta_->runtime_state.pad[offset]);
      return __atomic_load_n(value, __ATOMIC_ACQUIRE);
    };
    frame.launch_count = counter(3);
    frame.hit_count = counter(7);
    frame.sequence = image.seq; frame.timestamp_ns = image.timestamp_ns;
    const auto now = Clock::now();
    frame.received_monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    map_time(frame, now);
    ++received_;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      // The consumer must see a discontinuity even if its first frame after a
      // restart is replaced by a newer frame before read().
      if (latest_) {
        if (latest_->discontinuity && latest_->generation == frame.generation) frame.discontinuity = true;
        recycle(latest_->bgr); ++replaced_;
      }
      latest_ = std::move(frame);
    }
    frame_ready_.notify_one();
    return true;
  }
  void run() {
    while (!stop_.load()) {
      bool progress = false;
      try {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!connected_.load()) connect();
        if (connected_.load()) {
          if (!healthy()) disconnect();
          else progress = receive();
        }
      } catch (...) {
        ++malformed_;
        std::lock_guard<std::mutex> lock(map_mutex_);
        disconnect();
      }
      if (!progress) {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        wake_.wait_for(lock, connected_.load() ? 1ms : 20ms, [&] { return stop_.load(); });
      }
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    disconnect();
  }

  const std::string meta_path_, pool_path_;
  const std::chrono::milliseconds stale_after_;
  Mapping meta_mapping_, pool_mapping_;
  talos::ShmMetaRegion * meta_ = nullptr;
  std::mutex map_mutex_, frame_mutex_, wait_mutex_;
  std::uint64_t last_fire_token_ = 0;  // Protected by map_mutex_.
  std::condition_variable frame_ready_, wake_;
  std::optional<SimFrame> latest_;
  std::vector<cv::Mat> spare_;
  std::atomic<bool> stop_{false}, connected_{false};
  std::atomic<std::uint64_t> received_{0}, replaced_{0}, mismatch_{0}, reconnects_{0};
  std::atomic<std::uint64_t> stale_{0}, clock_resets_{0}, malformed_{0};
  std::uint64_t created_ = 0, heartbeat_ = 0, generation_ = 0;
  std::uint64_t stale_created_ = 0, stale_heartbeat_ = 0;
  Clock::time_point heartbeat_changed_{};
  bool have_previous_ = false, have_mapped_ = false, pending_discontinuity_ = true;
  std::uint64_t previous_sequence_ = 0, previous_source_ = 0, source_anchor_ = 0;
  Clock::time_point previous_received_{}, previous_mapped_{}, steady_anchor_{};
  std::thread thread_;  // Declared last: worker observes fully initialized state.
};

Simulator::Simulator(std::string directory, std::chrono::milliseconds stale_after)
: impl_(std::make_unique<Impl>(std::move(directory), stale_after)) {}
Simulator::~Simulator() = default;
bool Simulator::read(SimFrame & frame, std::chrono::milliseconds timeout) { return impl_->read(frame, timeout); }
void Simulator::send(double yaw, double pitch, bool fire, double distance) { impl_->send(yaw, pitch, fire, distance); }
Stats Simulator::stats() const { return impl_->stats(); }
bool Simulator::connected() const { return impl_->connected(); }
}  // namespace sim_bridge
