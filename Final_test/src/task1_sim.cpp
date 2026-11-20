#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <list>
#include <cmath>
#include <algorithm>

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

  int frame_index=0;
  while (!exiter.exit()) {
    // Your code start
  //空帧检测
    if (!simulator.read(frame, std::chrono::milliseconds(10))) continue;
    if (frame.discontinuity) tracker.reset();
  
  //标记云台角度
    const auto & q=frame.q_world_gimbal; 
    const double yuntai_yaw = std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
    const double yuntai_pitch = std::asin(std::clamp(2.0 * (q.w()*q.y() - q.z()*q.x()), -1.0, 1.0));
  
  //相机参数的标定
    solver.set_camera_calibration(frame.camera_matrix(), frame.distortion_coeffs());
    solver.set_world_camera_pose(frame.world_camera_rotation(), frame.world_camera_position());
    solver.set_R_gimbal2world(q);

  //若没有读取到装甲板，则保持原有状态
    auto armors = yolo.detect(frame.bgr , frame_index++);
    // if (armors.empty()) {
    //   simulator.send(yuntai_yaw, yuntai_pitch, false, 0.0);
    //   continue;
    // }

  //逐个解算检测到的装甲板，成功的放进 std::list<Armor>
    std::list<auto_aim::Armor> solved;
    for(auto & a : armors){
      if(solver.try_solve(a)) solved.push_back(a);
    }

  //追踪目标，检查是否检测到
    auto target = tracker.track(solved, frame.timestamp);
    if (!target) {
      simulator.send(yuntai_yaw, -yuntai_pitch, false, 0.0);  // 没跟踪到目标
      continue;
  }

  //预击打
    auto_aim::PlanningContext fire;
    fire.observation_time = frame.timestamp;
    fire.muzzle_world = frame.p_world_gimbal + frame.q_world_gimbal * frame.t_gimbal_muzzle;
    const double age_s = sim_bridge::capture_age_ms(frame) * 1e-3;
    fire.prediction_delay_s = age_s + options.delay_s;
    auto plan = planner.plan(*target,options.bullet_speed, fire);
  
  //判断预击打是否成立
    const bool fresh = age_s <= options.max_age_ms * 1e-3;

    const bool tracking =(tracker.state()=="tracking");
    const bool coasting =(tracker.state()=="temporarily_lost");
    const bool trackering_state= tracking || coasting ;

    const bool control = options.control && plan.control && plan.solver_ok &&
                         trackering_state && fresh;
    if (control) {
      simulator.send(plan.yaw, plan.pitch, options.fire_enabled && plan.fire, plan.distance);
    } 
    else {
      simulator.send(yuntai_yaw, yuntai_pitch, false, 0.0);
    }

    //调试发布（PlotJuggler + Foxglove）
    if (plotter || foxglove) {
      nlohmann::json m;
      m["gimbal_yaw"] = yuntai_yaw;
      m["gimbal_pitch"] = yuntai_pitch;
      m["plan_yaw"] = plan.control ? plan.yaw : 0.0;
      m["plan_pitch"] = plan.control ? plan.pitch : 0.0;
      m["tracking"] = target.has_value();
      m["state"] = tracker.state();
      m["age_ms"] = sim_bridge::capture_age_ms(frame);

      if (plotter) plotter->plot(m);

      if (foxglove) {
        debug_frame.timestamp_ns = frame.timestamp_ns;
        debug_frame.telemetry = m;
        debug_frame.image = frame.bgr.clone();
        debug_frame.measured.clear();
        debug_frame.has_target = target.has_value();
        for (const auto & a : solved) {
          debug_frame.measured.push_back(a.xyz_in_world);
          tools::draw_points(debug_frame.image, a.points, {0, 255, 0}, 2);
        }
        foxglove->publish(debug_frame);
      }
    }
    // Your code end
  }

  return 0;
} catch (const std::exception &error) {
  std::cerr << "task1_sim: " << error.what() << '\n';
  return 1;
}
