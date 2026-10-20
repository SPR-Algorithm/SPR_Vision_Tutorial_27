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

  while (!exiter.exit()) {
    // Your code start

    // Your code end
  }

  return 0;
} catch (const std::exception &error) {
  std::cerr << "task1_sim: " << error.what() << '\n';
  return 1;
}
