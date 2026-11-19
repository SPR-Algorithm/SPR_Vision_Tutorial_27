#include <chrono>
#include <exception>
#include <iostream>
#include <list>
#include <memory>

#include "sim_bridge/foxglove_debug.hpp"
#include "sim_bridge/simulation_config.hpp"
#include "sim_bridge/simulator.hpp"
#include "task/auto_aim/debug_utils.hpp"
#include "task/auto_aim/planner/planner.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/tracker.hpp"
#include "task/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
try
{
  const auto options = sim_bridge::load_sim_options(argc, argv);
  const auto &config_path = options.config;

  // ---- Khởi tạo công cụ ----
  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter> plotter;
  if (options.plotjuggler_enabled)
    plotter = std::make_unique<tools::Plotter>(options.plot_host, options.plot_port);

  sim_bridge::Simulator simulator(options.shm_directory);

  std::unique_ptr<sim_bridge::FoxgloveDebug> foxglove;
  if (options.foxglove_enabled)
  {
    foxglove = std::make_unique<sim_bridge::FoxgloveDebug>(
        options.foxglove_host, static_cast<std::uint16_t>(options.port),
        options.foxglove_rate, options.foxglove_visual_rate);
  }

  // ---- Khởi tạo pipeline auto_aim ----
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path);
  auto_aim::Planner planner(config_path);

  sim_bridge::SimFrame frame;
  sim_bridge::DebugFrame debug_frame;

  const auto start_time = std::chrono::steady_clock::now();
  auto last_plot_time = start_time;
  int frame_count = 0;

  // ---- Heartbeat: cứ 2 s in 1 dòng để biết pipeline đang đứng ở khâu nào ----
  long n_frames = 0, n_stale = 0, n_armors = 0, n_solved = 0, hb_prev_frames = 0;
  double last_age_ms = 0.0;
  auto last_hb = start_time;
  auto heartbeat = [&]() {
    const auto t_now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(t_now - last_hb).count() < 2.0) return;
    last_hb = t_now;
    const long new_frames = n_frames - hb_prev_frames;
    hb_prev_frames = n_frames;
    tools::logger()->info("[hb] frames={} (+{}) stale={} detected={} solved={} last_age_ms={:.1f} tracker={}",
                          n_frames, new_frames, n_stale, n_armors, n_solved, last_age_ms, tracker.state());
    if (n_frames == 0)
      tools::logger()->warn("[hb] NO FRAMES RECEIVED from simulator: check shm_directory ({}), F5, is simulator running?",
                            options.shm_directory);
    else if (new_frames == 0)
      tools::logger()->warn("[hb] NO NEW FRAMES in 2 s: simulator stopped sending (VM too slow? F5?)");
    else if (n_stale == n_frames)
      tools::logger()->warn("[hb] ALL frames dropped because too old: age {:.1f} ms > max_frame_age_ms={:.1f}. "
                            "Increase sim.max_frame_age_ms in YAML, or check clock sync",
                            last_age_ms, static_cast<double>(options.max_age_ms));
    else if (n_armors == 0)
      tools::logger()->warn("[hb] YOLO ran but found no armors: check enemy_color, model, min_confidence");
    else if (n_solved == 0)
      tools::logger()->warn("[hb] YOLO found armors but PnP failed: check camera_matrix, R_camera2gimbal");
  };
  tools::logger()->info("Entering main loop, waiting for frames (shm: {})", options.shm_directory);

  while (!exiter.exit())
  {
    heartbeat();

    // ---- 1. Nhận frame (timeout 50 ms) ----
    if (!simulator.read(frame, 50ms)) continue;
    ++n_frames;

    auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start_time).count();
    if (options.duration > 0 && elapsed > options.duration) break;

    // Bỏ frame quá cũ: dữ liệu cũ làm sai bù trễ.
    last_age_ms = sim_bridge::capture_age_ms(frame, now);
    if (last_age_ms > options.max_age_ms)
    {
      ++n_stale;
      continue;
    }

    // Gián đoạn từ simulator (reset cảnh...) -> xóa mục tiêu.
    if (frame.discontinuity) tracker.reset();

    // ---- 2. Detect + PnP ----
    auto armors = yolo.detect(frame.bgr, frame_count++);
    n_armors += static_cast<long>(armors.size());

    solver.set_camera_calibration(frame.camera_matrix(), frame.distortion_coeffs());
    solver.set_world_camera_pose(frame.world_camera_rotation(), frame.world_camera_position());

    std::list<auto_aim::Armor> observations;
    for (auto &armor : armors)
      if (solver.try_solve(armor)) observations.push_back(armor);
    n_solved += static_cast<long>(observations.size());

    // ---- 3. Tracker: LUÔN gọi mỗi frame, kể cả khi observations rỗng (để coasting hoạt động) ----
    auto target_opt = tracker.track(observations, frame.timestamp);

    // Mất mục tiêu: chỉ đẩy ảnh lên Foxglove để vẫn quan sát được.
    if (!target_opt.has_value())
    {
      if (foxglove)
      {
        debug_frame.timestamp_ns = frame.timestamp_ns;
        debug_frame.image = frame.bgr.clone();
        debug_frame.has_target = false;
        debug_frame.center = frame.p_world_gimbal;
        debug_frame.measured.clear();
        debug_frame.predicted.clear();
        for (const auto &a : observations)
          if (a.xyz_in_world.allFinite()) debug_frame.measured.push_back(a.xyz_in_world);
        foxglove->publish(std::move(debug_frame));
      }
      continue;
    }
    auto &target = target_opt.value();

    // ---- 4. Planner ----
    // age_s tính SAU detect/solve/track: gồm cả thời gian xử lý, không chỉ tuổi lúc nhận.
    now = std::chrono::steady_clock::now();
    const double age_ms = sim_bridge::capture_age_ms(frame, now);

    auto_aim::PlanningContext context;
    context.observation_time = frame.timestamp;
    context.muzzle_world = frame.p_world_gimbal + frame.q_world_gimbal * frame.t_gimbal_muzzle;
    context.prediction_delay_s = age_ms / 1000.0 + options.delay_s;

    auto plan = planner.plan(target, options.bullet_speed, context);

    // ---- 5. Gửi lệnh. Bám mù (coasting) thì không bắn. ----
    if (options.control && plan.control)
    {
      const bool fire = options.fire_enabled && plan.fire && !tracker.coasting();
      simulator.send(plan.yaw, plan.pitch, fire, plan.distance);
    }

    // ---- 6. Dữ liệu gỡ lỗi (PlotJuggler) ----
    const double plot_dt = std::chrono::duration<double>(now - last_plot_time).count();
    if (plot_dt > 1.0 / options.plot_rate)
    {
      last_plot_time = now;
      nlohmann::json j;
      j["time"] = std::chrono::duration<double>(now - start_time).count();
      j["timestamp"] = frame.timestamp_ns / 1e9;
      j["tracker"]["state"] = tracker.state();
      j["tracker"]["coasting"] = tracker.coasting();
      j["plan"]["yaw"] = plan.yaw;
      j["plan"]["pitch"] = plan.pitch;
      j["plan"]["valid"] = plan.control;
      j["plan"]["fire"] = plan.fire;
      j["plan"]["error"] = plan.tracking_error;
      j["plan"]["fly_time"] = plan.fly_time;
      j["target"]["yaw"] = plan.target_yaw;
      j["target"]["pitch"] = plan.target_pitch;
      j["gimbal"]["yaw"] = auto_aim::debug::gimbal_yaw(frame.q_world_gimbal);
      j["gimbal"]["pitch"] = auto_aim::debug::gimbal_pitch(frame.q_world_gimbal);
      j["frame_age_ms"] = age_ms;
      auto_aim::debug::fill_json(j, observations, target, plan.aim_xyza, plan.aim_id);
      if (plotter) plotter->plot(j);
    }

    // ---- 7. Foxglove ----
    if (foxglove)
    {
      debug_frame.timestamp_ns = frame.timestamp_ns;
      debug_frame.image = frame.bgr.clone();
      debug_frame.has_target = plan.control;
      debug_frame.center = frame.p_world_gimbal;
      debug_frame.measured.clear();
      debug_frame.predicted.clear();

      for (const auto &armor : observations)
        if (armor.xyz_in_world.allFinite()) debug_frame.measured.push_back(armor.xyz_in_world);

      // Dự đoán tại thời điểm đạn tới: trễ xử lý + thời gian bay.
      const double total_lead_s = context.prediction_delay_s + plan.fly_time;
      const auto impact_time =
          frame.timestamp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(total_lead_s));
      const auto pred = target.predict(impact_time);
      for (const auto &pa : pred.armors) debug_frame.predicted.push_back(pa.xyza.head<3>());

      nlohmann::json telemetry;
      telemetry["plan"]["yaw"] = plan.yaw;
      telemetry["plan"]["pitch"] = plan.pitch;
      telemetry["plan"]["valid"] = plan.control;
      telemetry["target"]["yaw"] = plan.target_yaw;
      telemetry["target"]["pitch"] = plan.target_pitch;
      telemetry["error"]["yaw"] = plan.tracking_error;
      telemetry["frame_age_ms"] = age_ms;
      telemetry["tracker"] = tracker.state();
      const auto tel = target.telemetry();
      telemetry["est"]["nis"] = tel.nis;
      telemetry["est"]["vyaw"] = tel.vyaw;
      debug_frame.telemetry = telemetry;

      foxglove->publish(std::move(debug_frame));
    }
  }

  return 0;
}
catch (const std::exception &error)
{
  std::cerr << "task1_sim: " << error.what() << '\n';
  return 1;
}
