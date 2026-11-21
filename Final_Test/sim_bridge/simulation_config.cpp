#include "simulation_config.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace sim_bridge {
namespace {

struct CommandLine {
  std::string config = std::string(PROJECT_SOURCE_PATH) + "/configs/simulator.yaml";
  bool no_foxglove = false;
  bool no_plotjuggler = false;
  double duration = -1;
  int port = -1;
  int plot_port = -1;
};

CommandLine command_line(int argc, char **argv) {
  CommandLine out;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::invalid_argument("Missing value for " + arg);
      return argv[i];
    };
    if (arg == "--config") out.config = value();
    else if (arg == "--duration") out.duration = std::stod(value());
    else if (arg == "--foxglove-port") out.port = std::stoi(value());
    else if (arg == "--plotjuggler-port") out.plot_port = std::stoi(value());
    else if (arg == "--no-foxglove") out.no_foxglove = true;
    else if (arg == "--no-plotjuggler") out.no_plotjuggler = true;
    else if (arg == "--help" || arg == "-h") {
      std::cout << "task1_sim [--config PATH] [--duration SECONDS] [--foxglove-port PORT]\n"
                   "          [--plotjuggler-port PORT] [--no-plotjuggler]\n"
                   "          [--no-foxglove]\n"
                   "Bevy default Talos IPC: /tmp/talos_ipc_{meta,image_pool}.\n"
                   "Foxglove connection: ws://127.0.0.1:8765 . Ctrl+C exits.\n"
                   "PlotJuggler UDP server + JSON parser: 127.0.0.1:9870 .\n"
                   "Student template: implement the pipeline in src/task1_sim.cpp.\n";
      std::exit(0);
    } else if (!arg.empty() && arg[0] != '-') out.config = arg;
    else throw std::invalid_argument("Unknown argument: " + arg);
  }
  if (!std::isfinite(out.duration)) throw std::invalid_argument("Invalid duration");
  if (out.port > 65535 || out.port < -1) throw std::invalid_argument("Invalid Foxglove port");
  if (out.plot_port > 65535 || out.plot_port == 0 || out.plot_port < -1)
    throw std::invalid_argument("Invalid PlotJuggler port");
  return out;
}

}  // namespace

SimOptions load_sim_options(int argc, char **argv) {
  const auto cli = command_line(argc, argv);
  const auto yaml = YAML::LoadFile(cli.config);
  const auto sim = yaml["sim"];
  const auto fox = yaml["foxglove"];
  const auto plot = yaml["plotjuggler"];

  SimOptions out;
  out.config = cli.config;
  out.duration = cli.duration >= 0 ? cli.duration : sim["duration_s"].as<double>(0);
  out.max_age_ms = sim["max_frame_age_ms"].as<double>(100);
  out.bullet_speed = sim["bullet_speed"].as<double>(25);
  out.delay_s = sim["execution_delay_ms"].as<double>(0) / 1000;
  out.control = sim["control"].as<bool>(true);
  out.fire_enabled = sim["fire"].as<bool>(true);
  if (!std::isfinite(out.duration) || out.duration < 0 || !std::isfinite(out.max_age_ms) ||
      !std::isfinite(out.bullet_speed) || !std::isfinite(out.delay_s) ||
      out.max_age_ms <= 0 || out.bullet_speed <= 0 || out.delay_s < 0)
    throw std::invalid_argument("Invalid simulation timing/ballistics configuration");

  out.shm_directory = sim["shm_directory"].as<std::string>("/tmp");
  out.enemy_color = yaml["enemy_color"].as<std::string>("blue");
  out.only_outpost = yaml["only_outpost"].as<bool>(true);
  out.foxglove_enabled = !cli.no_foxglove && fox["enabled"].as<bool>(true);
  if (out.foxglove_enabled) {
    out.port = cli.port >= 0 ? cli.port : fox["port"].as<int>(8765);
    if (out.port < 0 || out.port > 65535) throw std::invalid_argument("Invalid Foxglove port");
    out.foxglove_host = fox["host"].as<std::string>("127.0.0.1");
    out.foxglove_rate = fox["rate_hz"].as<double>(50);
    out.foxglove_visual_rate = fox["visual_rate_hz"].as<double>(10);
  }

  out.plot_rate = plot["rate_hz"].as<double>(100);
  out.plotjuggler_enabled = !cli.no_plotjuggler &&
    (cli.plot_port > 0 || plot["enabled"].as<bool>(false));
  if (out.plotjuggler_enabled) {
    if (!std::isfinite(out.plot_rate) || out.plot_rate <= 0 || out.plot_rate > 1000)
      throw std::invalid_argument("PlotJuggler rate_hz must be in (0, 1000]");
    out.plot_port = cli.plot_port > 0 ? cli.plot_port : plot["port"].as<int>(9870);
    if (out.plot_port < 1 || out.plot_port > 65535)
      throw std::invalid_argument("Plotter UDP port must be in [1, 65535]");
    out.plot_host = plot["host"].as<std::string>("127.0.0.1");
  }
  return out;
}

}  // namespace sim_bridge
