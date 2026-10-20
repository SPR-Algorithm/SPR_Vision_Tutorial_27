#pragma once

#include <string>

namespace sim_bridge {

struct SimOptions {
  std::string config;
  double duration = 0;
  double max_age_ms = 100;
  double bullet_speed = 25;
  double delay_s = 0;
  double plot_rate = 100;
  bool control = true;
  bool fire_enabled = true;
  bool only_outpost = true;
  std::string shm_directory = "/tmp";
  std::string enemy_color = "blue";
  bool foxglove_enabled = true;
  std::string foxglove_host = "127.0.0.1";
  int port = 8765;
  double foxglove_rate = 50;
  double foxglove_visual_rate = 10;
  bool plotjuggler_enabled = false;
  std::string plot_host = "127.0.0.1";
  int plot_port = 9870;
};

SimOptions load_sim_options(int argc, char **argv);

}  // namespace sim_bridge
