#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

// Talos ABI v2. Mirror crates/talos-ipc/src/layout.rs, including implicit padding.
// Raw integer atomics use GCC/Clang __atomic builtins: a C++ std::atomic object
// is not constructed in memory owned by the Rust producer.
namespace sim_bridge::talos {
constexpr std::uint32_t kMagic = 0x54414C05;
constexpr std::uint32_t kVersion = 2;
constexpr std::uint32_t kWidth = 1440, kHeight = 1080;
constexpr std::size_t kImageSize = std::size_t(kWidth) * kHeight * 3;
constexpr std::size_t kPoolSize = kImageSize * 3;
constexpr std::uint8_t kNew = 0x80, kIndexMask = 0x03;
constexpr const char * kMetaName = "talos_ipc_meta";
constexpr const char * kPoolName = "talos_ipc_image_pool";

struct alignas(32) ImageMeta {
  std::uint64_t seq, timestamp_ns;
  std::uint32_t width, height;
  std::uint8_t buffer_id, format, pad[6];
};
struct alignas(64) PoseMeta {
  std::uint64_t frame_seq;
  float position[3], quaternion[4];  // Quaternion is wxyz.
  std::uint64_t timestamp_ns;
  std::uint8_t pad[16];
};
struct alignas(32) GimbalCmd {
  std::uint64_t timestamp_ns;
  float yaw_deg, pitch_deg, distance_m;
  // Companion extension: pad[0..8] = most recent fire request UNIX ns, LE64.
  // Original v2 clients use all-zero padding and fire_advice pulses.
  std::uint8_t fire_advice, pad[11];
};
struct alignas(64) CameraInfo {
  std::uint64_t timestamp_ns;
  double fx, fy, cx, cy, distortion[5];
  std::uint32_t width, height;
  std::uint8_t pad[24];
};
struct alignas(64) ChassisObservation {
  std::uint64_t frame_seq, timestamp_ns;
  float dt_s, v_body[2], wz_radps, wheel_linear_mps[4], wheel_angular_radps[4];
  float a_body[2], alpha_z_radps2, rpy_rad[3], gyro_xyz_radps[3], accel_xyz_mps2[3];
  std::uint8_t pad[16];
};
template <typename T> struct alignas(64) TripleBuffer {
  std::uint8_t state, write_idx, read_idx, pad[61];
  T slots[3];
};
using ImageTripleBuffer = TripleBuffer<ImageMeta>;
using PoseTripleBuffer = TripleBuffer<PoseMeta>;
using GimbalTripleBuffer = TripleBuffer<GimbalCmd>;
struct alignas(64) ShmHeader {
  std::uint32_t magic, version;
  std::uint64_t created_ns, heartbeat_ns;
  std::uint32_t image_width, image_height;
  std::uint8_t pad[32];
};
struct alignas(32) GroundTruthTarget {
  std::uint64_t frame_seq, timestamp_ns;
  std::uint8_t team, armor_label, is_outpost, pad1;
  float position[3], vyaw, yaw;
  std::uint8_t pad[24];
};
struct alignas(64) GroundTruthRune {
  std::uint64_t frame_seq, timestamp_ns;
  std::uint8_t team, rune_mode, mechanism_state, pad1;
  float r_center_odom[3], radius, current_angle, v_roll;
  std::int32_t direction;
  float sin_amplitude, sin_omega, sin_phase, sin_offset, relative_time;
  std::int32_t blade_id;
  std::uint8_t target_activations[5], pad[20];
};
struct alignas(64) GroundTruthBatch {
  std::uint64_t frame_seq, timestamp_ns;
  std::uint32_t target_count, rune_count;
  GroundTruthTarget targets[16];
  GroundTruthRune runes[4];
  std::uint8_t pad[64];
};
struct alignas(64) RuntimeState {
  std::uint64_t timestamp_ns;
  std::uint8_t following, pad[55];
};
struct ShmMetaRegion {
  ShmHeader header;
  ImageTripleBuffer image;
  PoseTripleBuffer poses[5];
  GimbalTripleBuffer gimbal_cmd;
  CameraInfo camera_info;
  ChassisObservation chassis_observation;
  GroundTruthBatch ground_truth;
  RuntimeState runtime_state;
};

#define TALOS_SIZE(T, N) static_assert(sizeof(T) == N && std::is_standard_layout<T>::value, #T " ABI size")
#define TALOS_OFFSET(T, F, N) static_assert(offsetof(T, F) == N, #T "." #F " ABI offset")
TALOS_SIZE(ImageMeta, 32);
TALOS_OFFSET(ImageMeta, seq, 0); TALOS_OFFSET(ImageMeta, timestamp_ns, 8);
TALOS_OFFSET(ImageMeta, width, 16); TALOS_OFFSET(ImageMeta, height, 20);
TALOS_OFFSET(ImageMeta, buffer_id, 24); TALOS_OFFSET(ImageMeta, format, 25); TALOS_OFFSET(ImageMeta, pad, 26);
TALOS_SIZE(PoseMeta, 64);
TALOS_OFFSET(PoseMeta, frame_seq, 0); TALOS_OFFSET(PoseMeta, position, 8);
TALOS_OFFSET(PoseMeta, quaternion, 20); TALOS_OFFSET(PoseMeta, timestamp_ns, 40); TALOS_OFFSET(PoseMeta, pad, 48);
TALOS_SIZE(GimbalCmd, 32);
TALOS_OFFSET(GimbalCmd, timestamp_ns, 0); TALOS_OFFSET(GimbalCmd, yaw_deg, 8);
TALOS_OFFSET(GimbalCmd, pitch_deg, 12); TALOS_OFFSET(GimbalCmd, distance_m, 16);
TALOS_OFFSET(GimbalCmd, fire_advice, 20); TALOS_OFFSET(GimbalCmd, pad, 21);
TALOS_SIZE(CameraInfo, 128);
TALOS_OFFSET(CameraInfo, timestamp_ns, 0); TALOS_OFFSET(CameraInfo, fx, 8);
TALOS_OFFSET(CameraInfo, fy, 16); TALOS_OFFSET(CameraInfo, cx, 24); TALOS_OFFSET(CameraInfo, cy, 32);
TALOS_OFFSET(CameraInfo, distortion, 40); TALOS_OFFSET(CameraInfo, width, 80);
TALOS_OFFSET(CameraInfo, height, 84); TALOS_OFFSET(CameraInfo, pad, 88);
TALOS_SIZE(ChassisObservation, 128);
TALOS_OFFSET(ChassisObservation, frame_seq, 0); TALOS_OFFSET(ChassisObservation, timestamp_ns, 8);
TALOS_OFFSET(ChassisObservation, dt_s, 16); TALOS_OFFSET(ChassisObservation, v_body, 20);
TALOS_OFFSET(ChassisObservation, wz_radps, 28); TALOS_OFFSET(ChassisObservation, wheel_linear_mps, 32);
TALOS_OFFSET(ChassisObservation, wheel_angular_radps, 48); TALOS_OFFSET(ChassisObservation, a_body, 64);
TALOS_OFFSET(ChassisObservation, alpha_z_radps2, 72); TALOS_OFFSET(ChassisObservation, rpy_rad, 76);
TALOS_OFFSET(ChassisObservation, gyro_xyz_radps, 88); TALOS_OFFSET(ChassisObservation, accel_xyz_mps2, 100);
TALOS_OFFSET(ChassisObservation, pad, 112);
TALOS_SIZE(ImageTripleBuffer, 192); TALOS_SIZE(PoseTripleBuffer, 256); TALOS_SIZE(GimbalTripleBuffer, 192);
#define TALOS_BUFFER(T) TALOS_OFFSET(T, state, 0); TALOS_OFFSET(T, write_idx, 1); TALOS_OFFSET(T, read_idx, 2); TALOS_OFFSET(T, pad, 3); TALOS_OFFSET(T, slots, 64)
TALOS_BUFFER(ImageTripleBuffer); TALOS_BUFFER(PoseTripleBuffer); TALOS_BUFFER(GimbalTripleBuffer);
TALOS_SIZE(ShmHeader, 64);
TALOS_OFFSET(ShmHeader, magic, 0); TALOS_OFFSET(ShmHeader, version, 4);
TALOS_OFFSET(ShmHeader, created_ns, 8); TALOS_OFFSET(ShmHeader, heartbeat_ns, 16);
TALOS_OFFSET(ShmHeader, image_width, 24); TALOS_OFFSET(ShmHeader, image_height, 28); TALOS_OFFSET(ShmHeader, pad, 32);
TALOS_SIZE(GroundTruthTarget, 64);
TALOS_OFFSET(GroundTruthTarget, frame_seq, 0); TALOS_OFFSET(GroundTruthTarget, timestamp_ns, 8);
TALOS_OFFSET(GroundTruthTarget, team, 16); TALOS_OFFSET(GroundTruthTarget, armor_label, 17);
TALOS_OFFSET(GroundTruthTarget, is_outpost, 18); TALOS_OFFSET(GroundTruthTarget, pad1, 19);
TALOS_OFFSET(GroundTruthTarget, position, 20); TALOS_OFFSET(GroundTruthTarget, vyaw, 32);
TALOS_OFFSET(GroundTruthTarget, yaw, 36); TALOS_OFFSET(GroundTruthTarget, pad, 40);
TALOS_SIZE(GroundTruthRune, 128);
TALOS_OFFSET(GroundTruthRune, frame_seq, 0); TALOS_OFFSET(GroundTruthRune, timestamp_ns, 8);
TALOS_OFFSET(GroundTruthRune, team, 16); TALOS_OFFSET(GroundTruthRune, rune_mode, 17);
TALOS_OFFSET(GroundTruthRune, mechanism_state, 18); TALOS_OFFSET(GroundTruthRune, pad1, 19);
TALOS_OFFSET(GroundTruthRune, r_center_odom, 20); TALOS_OFFSET(GroundTruthRune, radius, 32);
TALOS_OFFSET(GroundTruthRune, current_angle, 36); TALOS_OFFSET(GroundTruthRune, v_roll, 40);
TALOS_OFFSET(GroundTruthRune, direction, 44); TALOS_OFFSET(GroundTruthRune, sin_amplitude, 48);
TALOS_OFFSET(GroundTruthRune, sin_omega, 52); TALOS_OFFSET(GroundTruthRune, sin_phase, 56);
TALOS_OFFSET(GroundTruthRune, sin_offset, 60); TALOS_OFFSET(GroundTruthRune, relative_time, 64);
TALOS_OFFSET(GroundTruthRune, blade_id, 68); TALOS_OFFSET(GroundTruthRune, target_activations, 72);
TALOS_OFFSET(GroundTruthRune, pad, 77);
TALOS_SIZE(GroundTruthBatch, 1664);
TALOS_OFFSET(GroundTruthBatch, frame_seq, 0); TALOS_OFFSET(GroundTruthBatch, timestamp_ns, 8);
TALOS_OFFSET(GroundTruthBatch, target_count, 16); TALOS_OFFSET(GroundTruthBatch, rune_count, 20);
TALOS_OFFSET(GroundTruthBatch, targets, 32); TALOS_OFFSET(GroundTruthBatch, runes, 1088); TALOS_OFFSET(GroundTruthBatch, pad, 1600);
TALOS_SIZE(RuntimeState, 64);
TALOS_OFFSET(RuntimeState, timestamp_ns, 0); TALOS_OFFSET(RuntimeState, following, 8); TALOS_OFFSET(RuntimeState, pad, 9);
TALOS_SIZE(ShmMetaRegion, 3712);
TALOS_OFFSET(ShmMetaRegion, header, 0); TALOS_OFFSET(ShmMetaRegion, image, 64);
TALOS_OFFSET(ShmMetaRegion, poses, 256); TALOS_OFFSET(ShmMetaRegion, gimbal_cmd, 1536);
TALOS_OFFSET(ShmMetaRegion, camera_info, 1728); TALOS_OFFSET(ShmMetaRegion, chassis_observation, 1856);
TALOS_OFFSET(ShmMetaRegion, ground_truth, 1984); TALOS_OFFSET(ShmMetaRegion, runtime_state, 3648);
static_assert(__atomic_always_lock_free(1, nullptr), "Talos requires lock-free byte atomics");
#undef TALOS_BUFFER
#undef TALOS_OFFSET
#undef TALOS_SIZE
}  // namespace sim_bridge::talos
