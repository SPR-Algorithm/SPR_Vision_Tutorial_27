#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <list>
#include <optional>
#include <string>
#include <chrono>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim
{
class Tracker
{
public:
  explicit Tracker(const std::string & config_path);

  void reset();
  std::string state() const;

  // 调用者负责先用 Solver 获得同一世界坐标系下的观测。
  // timestamp 使用图像采集时刻；无可用目标时返回 nullopt。
  // Người gọi chịu trách nhiệm dùng Solver để có observation trong cùng hệ tọa độ thế giới.
  // timestamp là thời điểm thu ảnh; trả về nullopt nếu không có mục tiêu khả dụng.
  std::optional<Target> track(const std::list<Armor> & observations, TimePoint timestamp);

private:
  Target target_;
  std::string state_ = "lost";
  // TODO(学生): 补充目标管理所需的数据。
  //TODO (sinh viên): bổ sung dữ liệu cần thiết cho việc quản lý mục tiêu.

  //init - frame dau tien nhin thay bang giap -> target khoi tao bo loc KF
  //tracking - neu cac frame tiep theo van nhin thay -> cap nhat du lieu moi lien tuc cho KF
  //lost - ko thay bang giap nua -> tracker phai dem so frame ko thay bang giap (1-2 frame, tracker ko xoa target; >15frame -> reset() de xoa bo loc KF )
  // thuc te de xac dinh lost ko nen dung dem so frame ma nen dung thoi gian (co the ket hop ca hai de tranh loi clock reset)

  TimePoint last_update_time_{}; // thời gian update frame gần nhất nhìn thấy bảng giáp
  // Ngưỡng thời gian tối đa (giây) không thấy target thì coi như mất (mặc định 0.5s)
  double max_lost_time_s_ = 0.5;
  int lost_frames_count_ =0;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
