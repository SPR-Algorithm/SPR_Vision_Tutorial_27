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

int main(int argc, char *argv[])
try
{
  const auto options = sim_bridge::load_sim_options(argc, argv);
  const auto &config_path = options.config;

  // 初始化工具类
  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter> plotter;
  if (options.plotjuggler_enabled)
  {
    plotter = std::make_unique<tools::Plotter>(options.plot_host, options.plot_port);
  }

  // 初始化仿真io类
  sim_bridge::Simulator simulator(options.shm_directory);
  std::unique_ptr<sim_bridge::FoxgloveDebug> foxglove;
  if (options.foxglove_enabled)
  {
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

  while (!exiter.exit())
  {
    // Your code start
    // 读取最新帧，超时50ms跳过
    if (!simulator.read(frame, std::chrono::milliseconds(50)))
      continue;
    // 跳过超过最大允许帧龄的过期帧
    if (sim_bridge::capture_age_ms(frame) > options.max_age_ms)
      continue;

    // 把当前帧的相机标定和位姿喂给PnP解算器，注意这里camera_matrix是函数要加()调用
    solver.set_camera_calibration(frame.camera_matrix(), frame.distortion_coeffs());
    solver.set_world_camera_pose(frame.world_camera_rotation(), frame.world_camera_position());

    // YOLO检测装甲板
    auto armors = yolo.detect(frame.bgr);
    // PnP解算得到世界坐标系下的装甲板3D位置
    std::list<auto_aim::Armor> solved_armors;
    for (auto &armor : armors)
    {
      if (solver.try_solve(armor))
      {
        solved_armors.push_back(armor);
      }
    }

    // 调用Tracker做目标关联、滤波更新
    auto target_opt = tracker.track(solved_armors, frame.timestamp);
    if (!target_opt.has_value())
      continue;
    auto &target = target_opt.value();

    // 填充规划上下文，调用MPC云台规划器
    auto_aim::PlanningContext ctx;
    ctx.observation_time = frame.timestamp;
    ctx.prediction_delay_s = 0.0; // 先给0延迟，后面你可以从yaml里读参数
    ctx.muzzle_world = frame.world_camera_position();
    auto plan = planner.plan(target, 25.0, ctx);

    // 合法规划结果才发送云台控制指令
    if (plan.control )
    {
      simulator.send(plan.yaw, plan.pitch, false, plan.distance);
    }

    // 把数据发布给PlotJuggler画对比曲线
    if (plotter)
    {
      nlohmann::json j;
      j["time"] = std::chrono::duration<double>(frame.timestamp.time_since_epoch()).count();
      if (!solved_armors.empty())
      {
        j["observation.x"] = solved_armors.front().xyz_in_world.x();
        j["observation.y"] = solved_armors.front().xyz_in_world.y();
        j["observation.z"] = solved_armors.front().xyz_in_world.z();
      }
      if (plan.control)
      {
        j["plan.yaw"] = plan.yaw;
        j["plan.pitch"] = plan.pitch;
      }
      plotter->plot(j);
    }

    // 把图像和检测结果发布给Foxglove做3D可视化
    if (foxglove)
    {
      debug_frame.image = frame.bgr;
      debug_frame.measured.clear();
      for (auto &armor : solved_armors)
      {
        debug_frame.measured.push_back(armor.xyz_in_world);
      }
      foxglove->publish(debug_frame);
    }

    // Your code end
  }

  return 0;
}
catch (const std::exception &error)
{
  std::cerr << "task1_sim: " << error.what() << '\n';
  return 1;
}
