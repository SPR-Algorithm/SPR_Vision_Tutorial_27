#include "tracker.hpp"

#include <vector>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path) : target_(config_path)
{
  try {
    auto yaml = YAML::LoadFile(config_path);
    max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>(max_temp_lost_count_);
    outpost_max_temp_lost_count_ =
      yaml["outpost_max_temp_lost_count"].as<int>(outpost_max_temp_lost_count_);
    outpost_mode_ = yaml["only_outpost"].as<bool>(false);
  } catch (const std::exception & e) {
    tools::logger()->warn("Tracker 读取参数失败，使用默认值: {}", e.what());
  }
}

void Tracker::reset()
{
  target_.reset();
  state_ = "lost";
  temp_lost_count_ = 0;
}

std::string Tracker::state() const
{
  return state_;
}

const Armor * Tracker::pick_best(const std::list<Armor> & observations) const
{
  // 题1 只有一个目标，直接挑置信度（YOLO 给的 confidence）最高的那个即可。
  const Armor * best = nullptr;
  for (const auto & obs : observations) {
    if (!obs.xyz_in_world.allFinite()) continue; // 跳过解算失败（NaN）的
    if (best == nullptr || obs.confidence > best->confidence) best = &obs;
  }
  return best;
}

std::optional<Target> Tracker::track(const std::list<Armor> & observations, TimePoint timestamp)
{
  // ================================================================
  // 先准备"这一帧要喂给 Target 的观测列表"。
  // 题1 只喂 1 块（挑置信度最高的）；题2 喂所有有效观测（多块板）。
  // ================================================================
  std::vector<Armor> to_update;
  if (outpost_mode_) {
    for (const auto & obs : observations) {
      if (obs.xyz_in_world.allFinite()) to_update.push_back(obs);
    }
  } else {
    const Armor * best = pick_best(observations);
    if (best != nullptr) to_update.push_back(*best);
  }

  // ================================================================
  // 这一帧没有可用观测（没检测到 / 全解算失败）
  // ================================================================
  if (to_update.empty()) {
    // 从来没建立过目标，谈不上"丢失"，直接返回空。
    if (state_ == "lost") return std::nullopt;

    // 连续丢帧计数 +1。
    temp_lost_count_++;
    // 前哨站允许丢更久的帧（板会旋转进出视野），用更大的上限。
    const int limit = outpost_mode_ ? outpost_max_temp_lost_count_ : max_temp_lost_count_;
    if (temp_lost_count_ > limit) {
      reset();
      return std::nullopt;
    }

    // 还在容忍范围内：目标只是"暂时丢了"。
    // 保留它的状态，让上层继续用旧估计外推（predict），不要立刻放弃。
    state_ = "temporary_lost";
    return target_;
  }

  // ================================================================
  // 有观测：喂给 Target（内部会自动完成初始化 / 预测 / 更新）
  // ================================================================
  target_.update(to_update, timestamp);

  temp_lost_count_ = 0;
  state_ = "tracking";
  return target_;
}
}  // namespace auto_aim
