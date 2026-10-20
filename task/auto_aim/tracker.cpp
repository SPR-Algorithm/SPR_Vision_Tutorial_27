#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_(solver), last_timestamp_(std::chrono::steady_clock::now())
{
  const auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = yaml["enemy_color"].as<std::string>("red") == "red" ? Color::red : Color::blue;
  min_detect_count_ = std::max(1, yaml["min_detect_count"].as<int>(3));
  normal_temp_lost_count_ = std::max(1, yaml["max_temp_lost_count"].as<int>(10));
  outpost_max_temp_lost_count_ = std::max(1, yaml["outpost_max_temp_lost_count"].as<int>(50));
  outpost_radius_ = yaml["outpost_radius"].as<double>(0.2765);
  if (!std::isfinite(outpost_radius_) || outpost_radius_ <= 0.05 || outpost_radius_ >= 0.5) {
    throw std::invalid_argument("outpost_radius must be finite and between 0.05 and 0.5 meters");
  }
  if (const auto target_config = yaml["target"];
      target_config && target_config["measurement_stddev_scale"]) {
    const auto node = target_config["measurement_stddev_scale"];
    if (node.IsScalar()) {
      measurement_stddev_scale_.setConstant(node.as<double>());
    } else if (node.IsSequence() && node.size() == 4) {
      for (int index = 0; index < 4; ++index) measurement_stddev_scale_[index] = node[index].as<double>();
    } else {
      throw std::invalid_argument("target.measurement_stddev_scale must be a scalar or four-value sequence");
    }
  }
  // Validate even before the first observation; invalid configuration must not
  // wait until target acquisition to fail. Every later acquisition gets it too.
  target_.set_measurement_stddev_scale(measurement_stddev_scale_);
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  const double dt = tools::delta_time(t, last_timestamp_);
  if (state_ != "lost" && dt < 0.0) return {};
  last_timestamp_ = t;
  if (state_ != "lost" && dt > 0.5) {
    tools::logger()->warn("[Tracker] Large frame gap: {:.3f}s", dt);
    state_ = "lost";
  }
  if (use_enemy_color) {
    armors.remove_if([&](const Armor & armor) { return armor.color != enemy_color_; });
  }
  armors.remove_if([](const Armor & armor) {
    return armor.name == ArmorName::not_armor || !std::isfinite(armor.center.x) ||
      !std::isfinite(armor.center.y);
  });
  // Stable sorting keeps image-center preference among equally important targets.
  armors.sort([](const Armor & a, const Armor & b) {
    const cv::Point2f image_center(720.0f, 540.0f);
    return cv::norm(a.center - image_center) < cv::norm(b.center - image_center);
  });
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  const bool found = state_ == "lost" ? set_target(armors, t) : update_target(armors, t);
  state_machine(found);
  if (state_ == "lost") return {};
  if (target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged");
    state_ = "lost";
    return {};
  }
  const auto & failures = target_.ekf().recent_nis_failures;
  if (failures.size() >= 20 &&
      std::accumulate(failures.begin(), failures.end(), 0) > 0.6 * failures.size()) {
    tools::logger()->debug("[Tracker] Repeated prior-innovation failures");
    state_ = "lost";
    return {};
  }
  return {target_};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;
    detect_count_ = 1;
    temp_lost_count_ = 0;
    state_ = min_detect_count_ <= 1 ? "tracking" : "detecting";
  } else if (state_ == "detecting") {
    if (!found) {
      detect_count_ = 0;
      state_ = "lost";
    } else if (++detect_count_ >= min_detect_count_) {
      state_ = "tracking";
    }
  } else if (state_ == "tracking") {
    if (!found) {
      temp_lost_count_ = 1;
      state_ = "temp_lost";
    }
  } else if (state_ == "temp_lost") {
    if (found) {
      temp_lost_count_ = 0;
      state_ = "tracking";
    } else {
      const int maximum_lost = target_.is_outpost() ?
        outpost_max_temp_lost_count_ : normal_temp_lost_count_;
      if (++temp_lost_count_ > maximum_lost) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  for (auto & armor : armors) {
    if (!solver_.try_solve(armor)) continue;
    const bool balance = armor.type == ArmorType::big &&
      (armor.name == ArmorName::three || armor.name == ArmorName::four || armor.name == ArmorName::five);
    if (armor.name == ArmorName::outpost) {
      Eigen::VectorXd P0(11);
      // cx vx cy vy z_ref vz theta omega r dh1 dh2
      P0 << 0.25, 0.04, 0.25, 0.04, 0.04, 0.04, 0.09, 9.0, 4e-4, 0.04, 0.04;
      target_ = Target(armor, t, outpost_radius_, 3, P0);
    } else if (armor.name == ArmorName::base) {
      target_ = Target(armor, t, 0.3205, 3);
    } else {
      target_ = Target(armor, t, 0.2, balance ? 2 : 4);
    }
    target_.set_measurement_stddev_scale(measurement_stddev_scale_);
    return true;
  }
  return false;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);
  std::vector<Armor> measurements;
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    if (solver_.try_solve(armor)) measurements.push_back(armor);
  }
  // A valid detection is not necessarily an accepted Kalman measurement.
  return target_.update(measurements) > 0;
}
}  // namespace auto_aim
