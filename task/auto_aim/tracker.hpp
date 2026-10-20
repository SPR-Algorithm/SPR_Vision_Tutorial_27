#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);
  std::string state() const;
  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_ = Color::red;
  int min_detect_count_ = 3;
  int detect_count_ = 0;
  int temp_lost_count_ = 0;
  int outpost_max_temp_lost_count_ = 50;
  int normal_temp_lost_count_ = 10;
  double outpost_radius_ = 0.2765;
  Eigen::Vector4d measurement_stddev_scale_ = Eigen::Vector4d::Ones();
  std::string state_ = "lost";
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;

  void state_machine(bool found);
  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
