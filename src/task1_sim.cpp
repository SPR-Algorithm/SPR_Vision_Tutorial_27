#include <chrono>
#include <exception>
#include <iostream>
#include <memory>

#include "sim_bridge/foxglove_debug.hpp"
#include "sim_bridge/simulation_config.hpp"
#include "sim_bridge/simulator.hpp"
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

int main(int argc, char *argv[]) try {
  const auto options = sim_bridge::load_sim_options(argc, argv);
  const auto &config_path = options.config;

  // 初始化工具类
  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter> plotter;
  if (options.plotjuggler_enabled) {
    plotter = std::make_unique<tools::Plotter>(options.plot_host, options.plot_port);
  }

  // 初始化仿真io类
  sim_bridge::Simulator simulator(options.shm_directory);
  std::unique_ptr<sim_bridge::FoxgloveDebug> foxglove;
  if (options.foxglove_enabled) {
    foxglove = std::make_unique<sim_bridge::FoxgloveDebug>(
      options.foxglove_host, static_cast<std::uint16_t>(options.port),
      options.foxglove_rate, options.foxglove_visual_rate);
  }

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path);
  auto_aim::Planner planner(config_path);

  sim_bridge::SimFrame frame;
  sim_bridge::DebugFrame debug_frame;
  
  auto start_time = std::chrono::steady_clock::now();//fixed value, set once
  auto last_plot_time = start_time; 
  int frame_count = 0;

  while (!exiter.exit()) {
    // Your code start

    // Read frame with 50ms timeout, skip iteration if none arrives
    if(!simulator.read(frame,50ms)) continue;

     //check duration limit
    auto now = std::chrono::steady_clock::now(); // new value each loop
    double elapsed = std::chrono::duration<double>(now - start_time).count();
    if (options.duration>0 && elapsed > options.duration){
      break;
    }

    //check frame age
    auto age_ms = sim_bridge::capture_age_ms(frame,now);
    if(age_ms > options.max_age_ms) continue;

    //detection
    auto armors = yolo.detect(frame.bgr, frame_count++);
    if (armors.empty()) {
      continue;
    }

    //set solver camera calibration and pose
    solver.set_camera_calibration(frame.camera_matrix(), frame.distortion_coeffs());
    Eigen::Quaterniond q_world_camera = frame.world_camera_rotation();
    Eigen::Vector3d p_world_camera = frame.world_camera_position();
    solver.set_world_camera_pose(q_world_camera, p_world_camera);

    //solve each armor
    std::list<auto_aim::Armor> observations;
    for (auto& armor : armors){
      if(solver.try_solve(armor)){
        observations.push_back(armor);
      }
    }
    if(observations.empty()) continue;

    //track
    if(frame.discontinuity) tracker.reset();
    auto target_opt = tracker.track(observations, frame.timestamp);
    if (!target_opt.has_value()){
      continue;
    }
    auto& target= target_opt.value();

     // build planning context
    auto_aim::PlanningContext context;
    context.observation_time = frame.timestamp;
    context.muzzle_world = frame.p_world_gimbal + frame.q_world_gimbal * frame.t_gimbal_muzzle;
    auto age_s = age_ms/1000.0;
    context.prediction_delay_s = age_s + options.delay_s;

    // plan
    double bullet_speed = options.bullet_speed;
    auto plan = planner.plan(target,bullet_speed,context);
    
    // send control
    if (options.control && plan.control) {
      bool fire = options.fire_enabled && plan.fire; 
      simulator.send(plan.yaw, plan.pitch, fire, plan.distance);
    } 

    // Publish debug data to PlotJuggler
    double plot_dt = std::chrono::duration<double>(now - last_plot_time).count();
    if (plot_dt > 1.0 / options.plot_rate){ //rate limiting
      last_plot_time = now; 
      nlohmann::json j;
      j["time"]      = std::chrono::duration<double>(now - start_time).count();
      j["timestamp"] = frame.timestamp_ns / 1e9; 
      j["plan"]["yaw"] = plan.yaw;
      j["plan"]["pitch"] = plan.pitch;
      j["plan"]["valid"] = plan.control;
      j["target"]["yaw"] = plan.target_yaw;
      j["target"]["pitch"] = plan.target_pitch;
      j["gimbal"]["yaw"] = frame.q_world_gimbal.toRotationMatrix().eulerAngles(2, 1, 0)[0];
      j["gimbal"]["pitch"] = frame.q_world_gimbal.toRotationMatrix().eulerAngles(2, 1, 0)[1];
      if (plotter) plotter->plot(j);
    }

    // Foxglove debug
    if (foxglove) {
      debug_frame.timestamp_ns = frame.timestamp_ns;
      debug_frame.image = frame.bgr.clone();
      debug_frame.has_target = plan.control;
      debug_frame.center = frame.p_world_gimbal;
      debug_frame.measured.clear();
      debug_frame.predicted.clear();

      // Collect measured armor positions
      for (const auto & armor : observations) {
        if (armor.xyz_in_world.allFinite()) {
          debug_frame.measured.push_back(armor.xyz_in_world);
        }
      }

      // Get prediction at impact time
      double processing_delay_s = context.prediction_delay_s;  // capture → execute
      double flight_time_s      = plan.fly_time;           // muzzle → target
      double total_lead_s       = processing_delay_s + flight_time_s;

      auto impact_time = frame.timestamp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(total_lead_s));
      auto pred = target.predict(impact_time);

      for (const auto & pa : pred.armors) {
        debug_frame.predicted.push_back(pa.xyza.head<3>()); // drop yaw
      }

      nlohmann::json telemetry;
      telemetry["plan"]["yaw"] = plan.yaw;
      telemetry["plan"]["pitch"] = plan.pitch;
      telemetry["plan"]["valid"] = plan.control;
      telemetry["target"]["yaw"] = plan.target_yaw;
      telemetry["target"]["pitch"] = plan.target_pitch;
      telemetry["error"]["yaw"] = plan.tracking_error;
      telemetry["frame_age_ms"] = age_ms;
      debug_frame.telemetry = telemetry;

      foxglove->publish(std::move(debug_frame));
    }

    // Your code end
  }

  return 0;
} catch (const std::exception &error) {
  std::cerr << "task1_sim: " << error.what() << '\n';
  return 1;
}
