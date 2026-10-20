#include "sim_bridge/simulator.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
namespace abi = sim_bridge::talos;
void check(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}
std::uint64_t wall_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
template <typename Predicate> void eventually(Predicate predicate, const char * message) {
  const auto deadline = Clock::now() + 3s;
  while (!predicate()) {
    check(Clock::now() < deadline, message);
    std::this_thread::sleep_for(1ms);
  }
}
template <typename T> void initialize(abi::TripleBuffer<T> & b) {
  b.state = 1; b.write_idx = 0; b.read_idx = 2;
}
template <typename T> void publish(abi::TripleBuffer<T> & b, const T & value) {
  b.slots[b.write_idx] = value;
  const auto old = __atomic_exchange_n(
    &b.state, std::uint8_t(b.write_idx | abi::kNew), __ATOMIC_ACQ_REL);
  b.write_idx = old & abi::kIndexMask;
}
struct Directory {
  std::string path;
  Directory() {
    char name[] = "/tmp/target-simulator-test-XXXXXX";
    const auto * result = ::mkdtemp(name);
    check(result != nullptr, "mkdtemp"); path = result;
  }
  ~Directory() { ::rmdir(path.c_str()); }
};
class Producer {
public:
  explicit Producer(const std::string & directory) : directory_(directory) {
    meta = static_cast<abi::ShmMetaRegion *>(map(abi::kMetaName, sizeof(*meta), meta_fd_));
    pixels = static_cast<std::uint8_t *>(map(abi::kPoolName, abi::kPoolSize, pool_fd_));
    std::memset(meta, 0, sizeof(*meta));
    meta->header.magic = abi::kMagic; meta->header.version = abi::kVersion;
    meta->header.created_ns = wall_ns();
    meta->header.image_width = abi::kWidth; meta->header.image_height = abi::kHeight;
    initialize(meta->image); for (auto & p : meta->poses) initialize(p);
    initialize(meta->gimbal_cmd);
    meta->camera_info.fx = meta->camera_info.fy = 1303.67;
    meta->camera_info.cx = 720; meta->camera_info.cy = 540;
    meta->camera_info.width = abi::kWidth; meta->camera_info.height = abi::kHeight;
    meta->runtime_state.following = 1;
    meta->runtime_state.pad[3] = 0x34; meta->runtime_state.pad[4] = 0x12;
    meta->runtime_state.pad[7] = 0x78; meta->runtime_state.pad[8] = 0x56;
    heartbeat();
  }
  ~Producer() {
    ::unlink((directory_ + "/" + abi::kMetaName).c_str());
    ::unlink((directory_ + "/" + abi::kPoolName).c_str());
    ::munmap(meta, sizeof(*meta)); ::munmap(pixels, abi::kPoolSize);
    ::close(meta_fd_); ::close(pool_fd_);
  }
  void heartbeat() { __atomic_store_n(&meta->header.heartbeat_ns, wall_ns(), __ATOMIC_RELEASE); }
  bool acknowledged() const {
    if (__atomic_load_n(&meta->image.state, __ATOMIC_ACQUIRE) & abi::kNew) return false;
    for (int i = 0; i < 4; ++i)
      if (__atomic_load_n(&meta->poses[i].state, __ATOMIC_ACQUIRE) & abi::kNew) return false;
    return true;
  }
  void wait_ack() {
    eventually([&] { heartbeat(); return acknowledged(); }, "consumer did not ACK image and all four poses");
  }
  void frame(std::uint64_t sequence, std::uint64_t timestamp = 0, bool mismatch = false) {
    wait_ack();
    if (!timestamp) timestamp = wall_ns();
    const auto color = std::uint8_t(sequence % 251);
    auto * dst = pixels + next_buffer_ * abi::kImageSize;
    for (std::size_t i = 0; i < abi::kImageSize; i += 3) {
      dst[i] = color; dst[i + 1] = 17; dst[i + 2] = 91;
    }
    for (int i = 0; i < 4; ++i) {
      abi::PoseMeta p{};
      p.frame_seq = sequence + (mismatch && i == 3 ? 1 : 0);
      p.timestamp_ns = timestamp; p.quaternion[0] = 1;
      p.position[0] = static_cast<float>(sequence);
      p.position[1] = static_cast<float>(i); p.position[2] = 0.125f;
      if (i == 3) {
        p.quaternion[0] = std::cos(0.1f); p.quaternion[3] = std::sin(0.1f);
      }
      publish(meta->poses[i], p);
    }
    abi::ImageMeta image{};
    image.seq = sequence; image.timestamp_ns = timestamp;
    image.width = abi::kWidth; image.height = abi::kHeight;
    image.buffer_id = next_buffer_;
    publish(meta->image, image);
    next_buffer_ = (next_buffer_ + 1) % 3;
    heartbeat();
  }
  abi::ShmMetaRegion * meta = nullptr;
  std::uint8_t * pixels = nullptr;
private:
  void * map(const char * filename, std::size_t size, int & fd) {
    fd = ::open((directory_ + "/" + filename).c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
    check(fd >= 0, "create fixture mmap");
    check(::ftruncate(fd, static_cast<off_t>(size)) == 0, "size fixture mmap");
    auto * value = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    check(value != MAP_FAILED, "map fixture"); return value;
  }
  std::string directory_;
  int meta_fd_ = -1, pool_fd_ = -1;
  std::uint8_t next_buffer_ = 0;
};
void verify(const sim_bridge::SimFrame & f, std::uint64_t sequence) {
  check(f.sequence == sequence, "wrong sequence");
  check(f.bgr.rows == int(abi::kHeight) && f.bgr.cols == int(abi::kWidth), "wrong dimensions");
  const auto expected = cv::Vec3b(91, 17, std::uint8_t(sequence % 251));
  for (int y = 0; y < f.bgr.rows; y += 31)
    for (int x = 0; x < f.bgr.cols; x += 29)
      check(f.bgr.at<cv::Vec3b>(y, x) == expected, "pixel corruption / RGB-BGR mismatch");
  check(std::abs(f.p_world_gimbal.x() - double(sequence)) < 1e-6, "image / pose mismatch");
  check(std::abs(f.q_gimbal_camera.angularDistance(Eigen::Quaterniond(
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())))) < 1e-6, "relative camera rotation lost");
  check(f.following && f.camera.fx > 1000 && f.received_monotonic_ns > 0, "metadata missing");
  check(f.launch_count == 0x1234 && f.hit_count == 0x5678, "runtime counter offsets/endian");
}

void transport_test() {
  Directory directory;
  sim_bridge::Simulator simulator(directory.path, 5s);
  sim_bridge::SimFrame frame;
  check(!simulator.read(frame, 20ms), "read without producer must time out");
  auto producer = std::make_unique<Producer>(directory.path);
  eventually([&] { return simulator.connected(); }, "late producer connection");
  producer->frame(1);
  check(simulator.read(frame, 2s), "first frame"); verify(frame, 1);
  check(frame.discontinuity && frame.generation > 0, "initial generation missing");
  auto owned = frame.bgr;  // Deliberately retain storage across later read()/receive calls.
  const auto first_generation = frame.generation;
  for (std::uint64_t sequence = 2; sequence <= 28; ++sequence) {
    producer->frame(sequence); producer->wait_ack();
    // Adversarial immediate reuse after ACK detects pixel copy happening too late.
    std::memset(producer->pixels, 0xEE, abi::kPoolSize);
  }
  eventually([&] { return simulator.stats().received == 28; }, "continuous drain during slow consumer");
  check(simulator.read(frame, 2s), "latest frame"); verify(frame, 28);
  check(simulator.stats().replaced >= 25, "pending frames must be bounded/latest-only");
  check(owned.at<cv::Vec3b>(0, 0) == cv::Vec3b(91, 17, 1), "retained consumer storage overwritten");

  simulator.send(0.3, 0.2, true, 4.5);
  check(__atomic_load_n(&producer->meta->gimbal_cmd.state, __ATOMIC_ACQUIRE) & abi::kNew, "command not published");
  const auto command_index = producer->meta->gimbal_cmd.state & abi::kIndexMask;
  const auto command = producer->meta->gimbal_cmd.slots[command_index];
  constexpr double deg = 180.0 / 3.14159265358979323846;
  check(std::abs(command.yaw_deg - 0.3 * deg) < 1e-5 &&
        std::abs(command.pitch_deg + 0.2 * deg) < 1e-5, "command angle units / Talos pitch sign");
  check(command.fire_advice == 1 && command.distance_m == 4.5f && command.timestamp_ns > 0, "command metadata");
  const auto fire_token = [](const abi::GimbalCmd & cmd) {
    std::uint64_t token = 0;
    for (int byte = 0; byte < 8; ++byte) token |= std::uint64_t(cmd.pad[byte]) << (byte * 8);
    return token;
  };
  check(fire_token(command) == command.timestamp_ns, "shot token must use request time");
  simulator.send(0.31, 0.21, false, 4.6);
  const auto next_index = producer->meta->gimbal_cmd.state & abi::kIndexMask;
  const auto overwritten = producer->meta->gimbal_cmd.slots[next_index];
  check(overwritten.fire_advice == 0 && fire_token(overwritten) == fire_token(command),
        "latest pose update erased the pending fire request");

  producer->frame(29, 0, true); producer->wait_ack();
  eventually([&] { return simulator.stats().mismatch == 1; }, "mismatched pose must be counted");
  check(!simulator.read(frame, 20ms), "mismatched frame exposed");
  auto last_time = frame.timestamp;
  producer->frame(2, wall_ns() - 2000000000ULL);
  check(simulator.read(frame, 2s), "sequence and source clock rollback recovery"); verify(frame, 2);
  check(frame.discontinuity && frame.generation > first_generation && frame.timestamp > last_time,
        "restart must reset generation without negative dt");
  producer->wait_ack();
  const auto old_generation = frame.generation;
  last_time = frame.timestamp;
  producer.reset();
  eventually([&] { return !simulator.connected(); }, "unlink should disconnect");
  producer = std::make_unique<Producer>(directory.path);
  eventually([&] { return simulator.connected(); }, "producer inode replacement reconnection");
  producer->frame(0);
  check(simulator.read(frame, 2s), "frame after reconnect"); verify(frame, 0);
  check(frame.discontinuity && frame.generation > old_generation && frame.timestamp > last_time,
        "reconnect must propagate reset");
}
void heartbeat_and_shutdown_test() {
  Directory directory;
  auto producer = std::make_unique<Producer>(directory.path);
  {
    sim_bridge::Simulator simulator(directory.path, 80ms);
    eventually([&] { return simulator.connected(); }, "heartbeat fixture connect");
    eventually([&] { return simulator.stats().stale_heartbeat == 1 && !simulator.connected(); },
               "stale heartbeat must disconnect");
    std::this_thread::sleep_for(100ms);
    check(!simulator.connected(), "unchanged stale producer must not reconnect");
    producer->heartbeat();
    eventually([&] { return simulator.connected(); }, "heartbeat resumes connection");
  }
  producer.reset();
  const auto before = Clock::now();
  { sim_bridge::Simulator absent(directory.path); }
  check(Clock::now() - before < 500ms, "shutdown must interrupt reconnect wait");
}
}  // namespace

int main() {
  try {
    transport_test(); heartbeat_and_shutdown_test();
    std::cout << "simulator mmap tests passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "simulator mmap test failure: " << e.what() << '\n'; return 1;
  }
}
