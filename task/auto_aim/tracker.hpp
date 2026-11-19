#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <list>
#include <optional>
#include <string>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim
{
  // Máy trạng thái vòng đời mục tiêu:
  //   Lost -> Detecting (đang xác nhận N frame) -> Tracking <-> TempLost (coasting) -> Lost
  class Tracker
  {
  public:
    explicit Tracker(const std::string &config_path);

    void reset();

    // "lost" | "detecting" | "tracking" | "temp_lost"
    std::string state() const;

    // true khi đang bám mù (chỉ predict, không có quan sát mới). Không nên bắn lúc này.
    bool coasting() const { return state_ == State::TempLost; }

    // PHẢI gọi mỗi frame, kể cả khi `observations` rỗng (để coasting và đếm thời gian mất hoạt động).
    // Trả về Target khi Tracking hoặc TempLost, ngược lại nullopt.
    std::optional<Target> track(const std::list<Armor> &observations, TimePoint timestamp);

  private:
    enum class State { Lost, Detecting, Tracking, TempLost };

    void set_state(State s, const char *why);
    void to_lost(const char *why);

    Target target_;
    State state_ = State::Lost;

    TimePoint last_update_time_{};
    int confirm_count_ = 0;
    int last_generation_ = 0;

    // tham số (YAML: tracker.max_lost_time_s, tracker.confirm_frames)
    double max_lost_time_s_ = 0.2;  // s, thời gian tối đa bám mù
    int confirm_frames_ = 3;        // số frame liên tiếp để xác nhận mục tiêu
  };
}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
