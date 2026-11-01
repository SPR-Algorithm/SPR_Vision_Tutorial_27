#include "tracker.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path) : target_(config_path) {}

void Tracker::reset()
{
  target_.reset();
  state_ = "lost";
  // TODO(学生): 清理自己增加的目标管理数据。
  last_update_time_ = TimePoint{};
  lost_frames_count_ = 0; //debug

}
std::string Tracker::state() const
{
  return state_;
}

std::optional<Target> Tracker::track(const std::list<Armor> & observations, TimePoint timestamp)
{
  // TODO(学生): 管理目标的建立、关联、更新和丢失，并调用 Target 接口。
  (void)observations;
  (void)timestamp;
  if (observations.empty()){
    if (state_=="tracking"){
      double time_since_last_seen = std::chrono::duration<double>(timestamp - last_update_time_).count();
      if (time_since_last_seen <= max_lost_time_s_){
        return target_; // vẫn trả về target_ để bám mù (coasting) (dự đoán quán tính)
      }
      else {
        state_= "lost";
        target_.reset();
        return std::nullopt;
      }
    }
    // Nếu state đang là "lost", trả về nullopt luôn
    return std::nullopt; // ham track nullopt -> target_opt.value() trong sim tra ve false -> ko goi planner -> ko gui lenh dieu khien
  }
  // Nếu có observation, cập nhật EKF
  std::vector<Armor> obs_vec(observations.begin(), observations.end());
  target_.update(obs_vec, timestamp);
  // Đánh dấu trạng thái
  state_ = "tracking"; // tu dau ko thay giap thi mac dinh ban dau state la lost, neu thay giap thi thanh tracking, dang tracking ma ko thay nua thi tra ve nullopt
  last_update_time_ = timestamp;
  return target_;
}
}  // namespace auto_aim





