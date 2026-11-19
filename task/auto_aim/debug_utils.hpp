#ifndef AUTO_AIM__DEBUG_UTILS_HPP
#define AUTO_AIM__DEBUG_UTILS_HPP

#include <Eigen/Dense>
#include <cmath>
#include <list>
#include <nlohmann/json.hpp>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim::debug
{
// KHÔNG dùng eulerAngles(2,1,0): Eigen trả về nghiệm tương đương (yaw lệch π, pitch/roll đảo dấu)
// khiến đồ thị nhảy lung tung. Dùng atan2 trên ma trận quay.
// Hệ: x trước, y trái, z lên (R cột 0 = hướng trước của gimbal trong world).
inline double gimbal_yaw(const Eigen::Quaterniond &q)
{
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

// Dấu của pitch: dương = ngẩng lên theo công thức này. Đối chiếu với quy ước của Planner/Gimbal
// (nếu ngược thì đổi dấu ở ĐÂY, chỉ ảnh hưởng đồ thị).
inline double gimbal_pitch(const Eigen::Quaterniond &q)
{
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(2, 0), std::hypot(R(0, 0), R(1, 0)));
}

// Ghi các đại lượng cần cho báo cáo: quan sát thô, ước lượng, NIS, điểm ngắm của planner.
inline void fill_json(nlohmann::json &j, const std::list<Armor> &observations, const Target &target,
                      const Eigen::Vector4d &aim_xyza, int aim_id)
{
  if (!observations.empty())
  {
    // Tấm gần nhất làm đại diện (khớp cách Target khởi tạo).
    const Armor *best = &observations.front();
    for (const auto &a : observations)
      if (a.xyz_in_world.norm() < best->xyz_in_world.norm()) best = &a;
    j["obs"]["x"] = best->xyz_in_world.x();
    j["obs"]["y"] = best->xyz_in_world.y();
    j["obs"]["z"] = best->xyz_in_world.z();
    j["obs"]["yaw"] = best->ypr_in_world[0];
    j["obs"]["count"] = static_cast<int>(observations.size());
  }
  const TargetTelemetry t = target.telemetry();
  if (t.valid)
  {
    j["est"]["x"] = t.pos.x();
    j["est"]["y"] = t.pos.y();
    j["est"]["z"] = t.pos.z();
    j["est"]["vx"] = t.vel.x();
    j["est"]["vy"] = t.vel.y();
    j["est"]["vz"] = t.vel.z();
    j["est"]["yaw"] = t.yaw;     // outpost: theta
    j["est"]["vyaw"] = t.vyaw;   // outpost: omega
    j["est"]["radius"] = t.radius;
    j["est"]["nis"] = t.nis;
    j["est"]["slot"] = t.matched_slot;
    j["est"]["matched"] = t.num_matched;
  }
  j["aim"]["x"] = aim_xyza.x();
  j["aim"]["y"] = aim_xyza.y();
  j["aim"]["z"] = aim_xyza.z();
  j["aim"]["yaw"] = aim_xyza.w();
  j["aim"]["id"] = aim_id;
}
}  // namespace auto_aim::debug

#endif  // AUTO_AIM__DEBUG_UTILS_HPP
