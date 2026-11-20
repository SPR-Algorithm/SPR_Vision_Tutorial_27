#include "tracker.hpp"

#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace auto_aim
{

namespace
{
template <typename T>
T setting(const YAML::Node & yaml, const std::string & key, const T & fallback)
{
  return yaml[key] ? yaml[key].as<T>() : fallback;
}
}  // namespace 检查该键在ymal里是否存在，若存在则取出来并转换为T

Tracker::Tracker(const std::string & config_path) : target_(config_path)  //构造函数，初始化ymal,读取相关配置
{
  const auto yaml = YAML::LoadFile(config_path);
  const auto t = yaml["target"];
  only_outpost_ = setting<bool>(yaml, "only_outpost", false);
  min_detect_count_ = setting<int>(yaml, "min_detect_count", 3);
  // 优先读 target.max_lost，退回模板的 max_temp_lost_count
  max_lost_count_ = (t && t["max_lost"]) ? t["max_lost"].as<int>()
                                         : setting<int>(yaml, "max_temp_lost_count", 15);
  outpost_max_lost_count_ = setting<int>(yaml, "outpost_max_temp_lost_count", 75);
  max_coast_s_ = setting<double>(yaml, "max_coast_s", 0.30);
  reset_gap_s_ = setting<double>(yaml, "reset_gap_s", 1.00);

  if (min_detect_count_ < 1) throw std::invalid_argument("min_detect_count must be >= 1");
}

void Tracker::reset()
{
  target_.reset();
  state_ = "lost";
  detect_count_ = 0;
  lost_count_ = 0;
  last_time_ = TimePoint{};
  last_seen_ = TimePoint{};
  // TODO(学生): 清理自己增加的目标管理数据。
}

std::string Tracker::state() const { return state_; }

void Tracker::select(const std::list<Armor> & observations, std::vector<Armor> & selected) const
{
  selected.clear();
  for (const auto & armor : observations) {
    if (!armor.xyz_in_world.allFinite() || !std::isfinite(armor.ypr_in_world[0])) continue;
    if (armor.name == ArmorName::not_armor || armor.name == ArmorName::base) continue;
    if (only_outpost_ && armor.name != ArmorName::outpost) continue;
    // 目标建立后只保留同类型，避免前哨站与步兵互污染
    if (target_.initialized() && (armor.name == ArmorName::outpost) != target_.isOutpost()) continue;
    selected.push_back(armor);
  }
}
std::optional<Target> Tracker::track(const std::list<Armor> & observations, TimePoint timestamp)
{
  if (timestamp == TimePoint{}) return std::nullopt;

  // 时间断裂（重启/暂停/跳变）
  if (last_time_ != TimePoint{}) {
    const double dt = std::chrono::duration<double>(timestamp - last_time_).count();
    if (dt < 0.0 || dt > reset_gap_s_) {
      tools::logger()->warn("[Tracker] clock discontinuity, resetting");
      reset();
    }
  }
  last_time_ = timestamp;

  std::vector<Armor> selected;
  select(observations, selected);
  target_.update(selected, timestamp);  // 空观测也会推进预测

  if (!target_.initialized()) {
    state_ = "lost";
    detect_count_ = 0;
    lost_count_ = 0;
    return std::nullopt;
  }

  if (selected.empty()) { ++lost_count_; }
  else { ++detect_count_; lost_count_ = 0; last_seen_ = timestamp; }

  const int lost_limit = target_.isOutpost() ? outpost_max_lost_count_ : max_lost_count_;

  // 丢目标：连续丢帧过多，或太久没有观测
  const bool coast_too_long = last_seen_ != TimePoint{} &&
    std::chrono::duration<double>(timestamp - last_seen_).count() > max_coast_s_;
  if (lost_count_ > lost_limit || coast_too_long) {
    tools::logger()->info("[Tracker] target lost");
    reset();
    return std::nullopt;
  }

  if (lost_count_ > 0) {
    state_ = "temporarily_lost";  // 仍输出，供 Planner 短时预测
  } else if (detect_count_ < min_detect_count_) {
    state_ = "detecting";
    return std::nullopt;  // 未确认前不输出
  } else {
    state_ = "tracking";
  }
  return target_;
  // TODO(学生): 管理目标的建立、关联、更新和丢失，并调用 Target 接口。
  // (void)observations;
  // (void)timestamp;
  // return std::nullopt;
}
}  // namespace auto_aim
