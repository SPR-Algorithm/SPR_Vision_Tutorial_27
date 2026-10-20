#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "sim_bridge/foxglove_debug.hpp"
#include "sim_bridge/simulator.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/planner/planner.hpp"
#include "task/auto_aim/shooter.hpp"
#include "task_sim.hpp"
#include "task/auto_aim/tracker.hpp"
#ifdef AUTO_AIM_HAS_OPENVINO
#include "task/auto_aim/yolo.hpp"
#endif
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace {
struct Options {
  std::string config = std::string(PROJECT_SOURCE_PATH) + "/configs/simulator.yaml";
  bool synthetic = false, image_only = false, no_foxglove = false, no_plotjuggler = false;
  double duration = -1;
  int port = -1;
  int plot_port = -1;
};
Options options(int argc, char **argv) {
  Options out;
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
    else if (arg == "--synthetic") out.synthetic = true;
    else if (arg == "--image-only") out.image_only = true;
    else if (arg == "--no-foxglove") out.no_foxglove = true;
    else if (arg == "--no-plotjuggler") out.no_plotjuggler = true;
    else if (arg == "--help" || arg == "-h") {
      std::cout << "task1_sim [--config PATH] [--duration SECONDS] [--foxglove-port PORT]\n"
                   "          [--plotjuggler-port PORT] [--no-plotjuggler]\n"
                   "          [--synthetic] [--image-only] [--no-foxglove]\n"
                   "Bevy default Talos IPC: /tmp/talos_ipc_{meta,image_pool}.\n"
                   "Foxglove connection: ws://127.0.0.1:8765 . Ctrl+C exits.\n"
                   "PlotJuggler UDP server + JSON parser: 127.0.0.1:9870 .\n"
                   "--synthetic runs a deterministic projected outpost/PnP smoke test.\n";
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
std::uint64_t unix_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
double milliseconds(Clock::duration dt) {
  return std::chrono::duration<double, std::milli>(dt).count();
}
double capture_age_ms(const sim_bridge::SimFrame &frame, Clock::time_point now) {
  double age = std::max(0.0, milliseconds(now - frame.timestamp));
  // Preserve actual capture age across epoch/sequence resets. The transport's
  // monotonic timestamp may be shifted to maintain order during a restart.
  const auto wall = unix_ns();
  if (frame.timestamp_ns > 0 && wall >= frame.timestamp_ns)
    age = std::max(age, (wall - frame.timestamp_ns) / 1e6);
  return age;
}
void calibration(auto_aim::Solver &solver, const sim_bridge::SimFrame &frame) {
  Eigen::Matrix3d K;
  K << frame.camera.fx, 0, frame.camera.cx, 0, frame.camera.fy, frame.camera.cy, 0, 0, 1;
  Eigen::Matrix<double, 5, 1> D;
  for (int i = 0; i < 5; ++i) D[i] = frame.camera.distortion[i];
  solver.set_camera_calibration(K, D);
  solver.set_world_camera_pose(
    frame.q_world_gimbal * frame.q_gimbal_camera,
    frame.p_world_gimbal + frame.q_world_gimbal * frame.t_gimbal_camera);
}
std::list<auto_aim::Armor> synthetic_frame(
  sim_bridge::SimFrame &frame, auto_aim::Solver &solver, std::uint64_t seq, double t,
  auto_aim::Color color) {
  frame.sequence = seq;
  frame.timestamp = Clock::now();
  frame.timestamp_ns = unix_ns();
  frame.received_monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    frame.timestamp.time_since_epoch()).count();
  frame.camera.fx = frame.camera.fy = 1303.67532368;
  frame.camera.cx = 720;
  frame.camera.cy = 540;
  frame.camera.width = 1440;
  frame.camera.height = 1080;
  frame.p_world_gimbal = {0, 0, 1.2};
  frame.following = true;
  frame.bgr = cv::Mat(1080, 1440, CV_8UC3, cv::Scalar(12, 12, 12));
  calibration(solver, frame);
  const double w = 0.8 * CV_PI;
  const double phase = t < 4 ? w * t : t < 6 ? w * 4 : w * (10 - t);
  const double heights[] = {1.165, 1.267, 1.369};
  std::list<auto_aim::Armor> result;
  for (int i = 0; i < 3; ++i) {
    const double a = tools::limit_rad(phase + i * 2 * CV_PI / 3);
    const Eigen::Vector3d p{6 - 0.275 * std::cos(a), -0.275 * std::sin(a), heights[i]};
    if (std::abs(tools::limit_rad(a - std::atan2(p.y(), p.x()))) > 75 * CV_PI / 180) continue;
    auto corners = solver.reproject_armor(p, a, auto_aim::small, auto_aim::outpost);
    std::vector<cv::Point> polygon;
    for (const auto &corner : corners) polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
    cv::polylines(frame.bgr, polygon, true, color == auto_aim::red ? cv::Scalar(0, 0, 255)
                                                                            : cv::Scalar(255, 0, 0), 3);
    result.emplace_back(color == auto_aim::blue ? 0 : 1, 6, 1.0f, cv::boundingRect(corners), corners);
  }
  return result;
}
// Continuous yaw for plotting; raw control angles retain their [-pi, pi] convention.
struct ContinuousAngle {
  std::optional<double> previous;
  double value = 0;
  void reset() { previous.reset(); }
  double update(double wrapped, double reference = 0) {
    if (previous) value += tools::limit_rad(wrapped - *previous);
    else value = reference + tools::limit_rad(wrapped - reference);
    previous = wrapped;
    return value;
  }
};
nlohmann::json angle_fields(double yaw, double pitch, double continuous) {
  const double deg = 180 / CV_PI;
  return {{"yaw", yaw}, {"pitch", pitch}, {"yaw_deg", yaw * deg}, {"pitch_deg", pitch * deg},
          {"yaw_unwrapped", continuous}, {"yaw_unwrapped_deg", continuous * deg}};
}
struct Window {
  std::deque<double> samples;
  void add(double value) {
    samples.push_back(value);
    if (samples.size() > 1000) samples.pop_front();
  }
  double percentile(double p) const {
    if (samples.empty()) return 0;
    std::vector<double> sorted(samples.begin(), samples.end());
    std::sort(sorted.begin(), sorted.end());
    return sorted[static_cast<std::size_t>(std::ceil(p * (sorted.size() - 1)))];
  }
};
}  // namespace

int run_task1_sim(int argc, char **argv) try {
  const auto opt = options(argc, argv);
  const auto yaml = YAML::LoadFile(opt.config);
  const auto sim = yaml["sim"];
  const auto fox = yaml["foxglove"];
  const auto plot = yaml["plotjuggler"];
  const double duration = opt.duration >= 0 ? opt.duration : sim["duration_s"].as<double>(0);
  const double max_age_ms = sim["max_frame_age_ms"].as<double>(100);
  const double bullet_speed = sim["bullet_speed"].as<double>(25);
  const double delay_s = sim["execution_delay_ms"].as<double>(0) / 1000;
  const bool control = sim["control"].as<bool>(true);
  const bool fire_enabled = sim["fire"].as<bool>(true);
  if (!std::isfinite(duration) || duration < 0 || !std::isfinite(max_age_ms) ||
      !std::isfinite(bullet_speed) || !std::isfinite(delay_s) ||
      max_age_ms <= 0 || bullet_speed <= 0 || delay_s < 0)
    throw std::invalid_argument("Invalid simulation timing/ballistics configuration");
  tools::Exiter exiter;
  auto_aim::Solver solver(opt.config);
  auto_aim::Planner planner(opt.config);
  auto_aim::Shooter shooter(opt.config);
  auto tracker = std::make_unique<auto_aim::Tracker>(opt.config, solver);
  std::unique_ptr<sim_bridge::Simulator> transport;
  if (!opt.synthetic) transport = std::make_unique<sim_bridge::Simulator>(
    sim["shm_directory"].as<std::string>("/tmp"));
  std::unique_ptr<sim_bridge::FoxgloveDebug> debug;
  if (!opt.no_foxglove && fox["enabled"].as<bool>(true)) {
    const int port = opt.port >= 0 ? opt.port : fox["port"].as<int>(8765);
    if (port < 0 || port > 65535) throw std::invalid_argument("Invalid Foxglove port");
    debug = std::make_unique<sim_bridge::FoxgloveDebug>(
      fox["host"].as<std::string>("127.0.0.1"),
      static_cast<std::uint16_t>(port),
      fox["rate_hz"].as<double>(50), fox["visual_rate_hz"].as<double>(10));
    tools::logger()->info("Foxglove: ws://{}:{}", fox["host"].as<std::string>("127.0.0.1"), debug->port());
  }
  std::unique_ptr<tools::Plotter> plotter;
  const double plot_rate = plot["rate_hz"].as<double>(100);
  if (!opt.no_plotjuggler && (opt.plot_port > 0 || plot["enabled"].as<bool>(false))) {
    if (!std::isfinite(plot_rate) || plot_rate <= 0 || plot_rate > 1000)
      throw std::invalid_argument("PlotJuggler rate_hz must be in (0, 1000]");
    const int plot_port = opt.plot_port > 0 ? opt.plot_port : plot["port"].as<int>(9870);
    plotter = std::make_unique<tools::Plotter>(plot["host"].as<std::string>("127.0.0.1"), plot_port);
    tools::logger()->info("PlotJuggler UDP JSON: {}:{}", plot["host"].as<std::string>("127.0.0.1"),
                          plot_port);
  }
#ifdef AUTO_AIM_HAS_OPENVINO
  std::unique_ptr<auto_aim::YOLO> detector;
  if (!opt.synthetic && !opt.image_only) detector = std::make_unique<auto_aim::YOLO>(opt.config, false);
#else
  if (!opt.synthetic && !opt.image_only)
    throw std::runtime_error("OpenVINO unavailable; build with it or use --image-only/--synthetic");
#endif
  const auto color = yaml["enemy_color"].as<std::string>("blue") == "red" ? auto_aim::red : auto_aim::blue;
  const bool only_outpost = yaml["only_outpost"].as<bool>(true);
  const auto start = Clock::now();
  auto report_time = start;
  auto next_synthetic = start;
  auto next_plot = start;
  ContinuousAngle gimbal_yaw_curve, target_yaw_curve, phase_curve;
  int last_target_updates = -1;
  std::uint64_t frames = 0, stale = 0, shots_requested = 0;
  double age_sum = 0, processing_sum = 0;
  Window ages, processing_times;
  double age_p50 = 0, age_p95 = 0, age_p99 = 0, processing_p95 = 0;
  double planning_estimate_s = 0;
  tools::logger()->info("task1_sim started ({})", opt.synthetic ? "synthetic PnP feed" : "Bevy Talos IPC");
  sim_bridge::SimFrame frame;
  while (!exiter.exit()) {
    const auto begin = Clock::now();
    const double elapsed = std::chrono::duration<double>(begin - start).count();
    if (duration > 0 && elapsed >= duration) break;
    std::list<auto_aim::Armor> armors;
    if (opt.synthetic) {
      std::this_thread::sleep_until(next_synthetic);
      next_synthetic += 10ms;
      armors = synthetic_frame(frame, solver, frames,
        std::chrono::duration<double>(Clock::now() - start).count(), color);
    } else {
      if (!transport->read(frame, 100ms)) {
        shooter.reset();
        if (Clock::now() - report_time >= 2s) {
          tools::logger()->info("Waiting for Bevy Talos frames in {}", sim["shm_directory"].as<std::string>("/tmp"));
          report_time = Clock::now();
          if (debug) {
            sim_bridge::DebugFrame idle;
            idle.timestamp_ns = unix_ns();
            const auto stats = transport->stats();
            idle.telemetry = {{"connected", transport->connected()}, {"tracking", false},
              {"converged", false}, {"fire", false}, {"fire_reason", "not_tracking"},
              {"shots_requested", shots_requested}, {"launch_count", frame.launch_count},
              {"hit_count", frame.hit_count}, {"received", stats.received}, {"replaced", stats.replaced},
              {"mismatch", stats.mismatch}, {"stale", stale}, {"tracker_state", "waiting"}};
            debug->publish(std::move(idle));
          }
        }
        continue;
      }
      if (frame.discontinuity) {
        tracker = std::make_unique<auto_aim::Tracker>(opt.config, solver);
        shooter.reset();
        planning_estimate_s = 0;
        gimbal_yaw_curve.reset(); target_yaw_curve.reset(); phase_curve.reset();
        last_target_updates = -1;
      }
      calibration(solver, frame);
    }
    const auto process_start = Clock::now();
    const double age_ms = capture_age_ms(frame, process_start);
    if (age_ms > max_age_ms) { ++stale; shooter.reset(); continue; }
#ifdef AUTO_AIM_HAS_OPENVINO
    if (detector) armors = detector->detect(frame.bgr, static_cast<int>(frame.sequence % 2147483647));
#endif
    const std::size_t raw_detected = armors.size();
    std::vector<std::string> raw_names, raw_colors;
    std::vector<double> raw_confidences;
    for (const auto &armor : armors) {
      raw_names.push_back(auto_aim::ARMOR_NAMES.at(armor.name));
      raw_colors.push_back(auto_aim::COLORS.at(armor.color));
      raw_confidences.push_back(armor.confidence);
    }
    armors.remove_if([&](const auto_aim::Armor &armor) {
      return armor.color != color || (only_outpost && armor.name != auto_aim::outpost);
    });
    const std::size_t detected = armors.size();
    auto targets = opt.image_only ? std::list<auto_aim::Target>{} : tracker->track(armors, frame.timestamp);
    sim_bridge::DebugFrame visualization;
    visualization.timestamp_ns = frame.timestamp_ns;
    // The latest-frame transport owns its pixels, so retaining this Mat in the debug worker is safe.
    visualization.image = frame.bgr;
    for (const auto &armor : armors) {
      if (armor.xyz_in_world.allFinite()) visualization.measured.push_back(armor.xyz_in_world);
      std::vector<cv::Point> polygon;
      for (const auto &point : armor.points) polygon.emplace_back(cvRound(point.x), cvRound(point.y));
      if (polygon.size() == 4) cv::polylines(visualization.image, polygon, true, cv::Scalar(0, 255, 0), 1);
    }
    auto_aim::Plan plan;
    double planning_ms = 0;
    const Eigen::Vector3d gun_forward = frame.q_world_gimbal * Eigen::Vector3d::UnitX();
    const double gun_yaw = std::atan2(gun_forward.y(), gun_forward.x());
    const double gun_pitch = std::atan2(gun_forward.z(), gun_forward.head<2>().norm());
    auto_aim::FireInput fire_input;
    fire_input.following = frame.following;
    fire_input.control_enabled = control;
    fire_input.fire_enabled = fire_enabled;
    fire_input.gun_yaw = gun_yaw;
    fire_input.gun_pitch = gun_pitch;
    auto &data = visualization.telemetry;
    data = {{"frame_seq", frame.sequence}, {"generation", frame.generation},
      {"frame_age_ms", age_ms}, {"detected", detected}, {"stale", stale},
      {"raw_detected", raw_detected}, {"raw_names", raw_names},
      {"raw_colors", raw_colors}, {"raw_confidences", raw_confidences},
      {"tracking", !targets.empty()}, {"synthetic", opt.synthetic},
      {"connected", opt.synthetic || transport->connected()}, {"tracker_state", tracker->state()},
      {"converged", false}, {"command_sent", false}, {"following", frame.following},
      {"control_enabled", control}, {"fire_enabled", fire_enabled}, {"measurement_valid", false},
      {"time", std::chrono::duration<double>(process_start - start).count()},
      {"max_frame_age_ms", max_age_ms}, {"fire_interval_ms", shooter.config().fire_interval_s * 1000},
      {"fire_yaw_tolerance_deg", shooter.config().yaw_tolerance * 180 / CV_PI},
      {"fire_pitch_tolerance_deg", shooter.config().pitch_tolerance * 180 / CV_PI}};
    if (!targets.empty()) {
      const auto &target = targets.front();
      const auto x = target.ekf_x();
      if (target.update_count() < last_target_updates) phase_curve.reset();
      last_target_updates = target.update_count();
      visualization.has_target = true;
      visualization.center = {x[0], x[2], x[4]};
      for (const auto &plate : target.armor_xyza_list()) {
        visualization.predicted.push_back(plate.head<3>());
        const auto pixels = solver.world2pixel({cv::Point3f(plate[0], plate[1], plate[2])});
        if (!pixels.empty()) cv::circle(visualization.image, pixels[0], 5, {0, 255, 255}, 2);
      }
      data.update({{"converged", target.is_converged()}, {"state_dim", x.size()},
        {"center_x", x[0]}, {"center_y", x[2]}, {"center_z", x[4]}, {"yaw", x[6]},
        {"vyaw", x[7]}, {"radius", x[8]}, {"vz", x[5]}, {"dh1", x[9]}, {"dh2", x[10]},
        {"z0", x[4]}, {"z1", target.armor_position(x, 1).z()},
        {"z2", target.armor_num() > 2 ? target.armor_position(x, 2).z() : x[4]},
        {"nis", target.ekf().last_nis}, {"accepted", target.last_associations().size()},
        {"update_count", target.update_count()}});
      data["ekf_x"] = std::vector<double>(x.data(), x.data() + x.size());
      const Eigen::VectorXd diagonal = target.ekf().P.diagonal();
      data["P_diag"] = std::vector<double>(diagonal.data(), diagonal.data() + diagonal.size());
      const Eigen::Vector3d muzzle = frame.p_world_gimbal + frame.q_world_gimbal * frame.t_gimbal_muzzle;
      auto_aim::PlanningContext context;
      context.muzzle_world = muzzle;
      context.prediction_delay_s = capture_age_ms(frame, Clock::now()) / 1000 +
                                   delay_s + planning_estimate_s;
      context.yaw = gun_yaw;
      context.pitch = gun_pitch;
      const auto planning_start = Clock::now();
      plan = planner.plan(target, bullet_speed, context);
      planning_ms = milliseconds(Clock::now() - planning_start);
      // Account for bounded planner computation in the next execution-time
      // estimate. A sudden scheduling stall is still rejected by freshness.
      planning_estimate_s = 0.8 * planning_estimate_s + 0.2 * planning_ms / 1000;
      fire_input.tracking = tracker->state() == "tracking";
      fire_input.converged = target.is_converged();
      fire_input.measurement = !target.last_associations().empty();
      data["measurement_valid"] = fire_input.measurement;
      if (plan.control) {
        const auto pixels = solver.world2pixel({cv::Point3f(
          plan.aim_xyza.x(), plan.aim_xyza.y(), plan.aim_xyza.z())});
        if (!pixels.empty()) cv::drawMarker(visualization.image, pixels[0],
          cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 18, 2);
      }
    }
    // Recheck freshness after inference and planning, using the actual camera
    // feedback to gate each launch request. Commands use pitch up-positive.
    fire_input.plan = plan;
    fire_input.now = Clock::now();
    const double command_age_ms = capture_age_ms(frame, fire_input.now);
    fire_input.frame_age_s = command_age_ms / 1000;
    fire_input.control_enabled = control;
    auto_aim::FireDecision firing;
    if (command_age_ms > max_age_ms) {
      shooter.reset();
      firing.reason = auto_aim::FireReason::stale_frame;
    } else {
      firing = shooter.update(fire_input);
    }
    const bool send = control && frame.following && plan.control && plan.solver_ok &&
                      command_age_ms <= max_age_ms;
    if (send) {
      if (transport) transport->send(plan.yaw, plan.pitch, firing.fire, plan.distance);
      else {
        // Deterministic ideal actuator feedback, applied to the NEXT frame.
        // Synthetic launch counts are requests, not physical hit validation.
        frame.q_world_gimbal = Eigen::AngleAxisd(plan.yaw, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(-plan.pitch, Eigen::Vector3d::UnitY());
      }
      if (firing.fire) ++shots_requested;
      data["command_sent"] = true;
    }
    if (opt.synthetic) frame.launch_count = static_cast<std::uint32_t>(shots_requested);
    data.update({{"command_age_ms", command_age_ms}, {"planned_control", plan.control},
      {"solver_ok", plan.solver_ok}, {"planner_fire", plan.fire}, {"fire", send && firing.fire},
      {"fire_reason", auto_aim::to_string(firing.reason)},
      {"fire_yaw_error_deg", firing.yaw_error * 180 / CV_PI},
      {"fire_pitch_error_deg", firing.pitch_error * 180 / CV_PI},
      {"fire_cooldown_ms", firing.cooldown_remaining_s * 1000},
      {"fire_stable_ms", firing.stable_time_s * 1000}, {"shots_requested", shots_requested},
      {"launch_count", frame.launch_count}, {"hit_count", frame.hit_count},
      {"gun_yaw", gun_yaw}, {"gun_pitch", gun_pitch}, {"aim_id", plan.aim_id},
      {"target_yaw", plan.target_yaw}, {"target_pitch", plan.target_pitch},
      {"yaw_vel", plan.yaw_vel}, {"pitch_vel", plan.pitch_vel},
      {"yaw_acc", plan.yaw_acc}, {"pitch_acc", plan.pitch_acc},
      {"aim_visibility_deg", plan.aim_visibility_angle * 180 / CV_PI},
      {"plan_tracking_error", plan.tracking_error}, {"distance", plan.distance},
      {"fly_time_ms", plan.fly_time * 1000}, {"planning_ms", planning_ms},
      {"planning_estimate_ms", planning_estimate_s * 1000}});
    data["aim_xyza"] = std::vector<double>(plan.aim_xyza.data(), plan.aim_xyza.data() + 4);
    const bool valid_plan = plan.control && plan.solver_ok;
    const double continuous_gimbal = gimbal_yaw_curve.update(gun_yaw, gun_yaw);
    data["gimbal"] = angle_fields(gun_yaw, gun_pitch, continuous_gimbal);
    data["gimbal"]["valid"] = true;
    data["target"] = {{"valid", valid_plan}, {"state_valid", !targets.empty()}};
    data["plan"] = {{"valid", valid_plan}};
    data["error"] = {{"valid", valid_plan}};
    if (valid_plan) {
      const double continuous_target = target_yaw_curve.update(plan.target_yaw, continuous_gimbal);
      data["target"].update(angle_fields(plan.target_yaw, plan.target_pitch, continuous_target));
      const double continuous_plan = continuous_target + tools::limit_rad(plan.yaw - plan.target_yaw);
      data["plan"].update(angle_fields(plan.yaw, plan.pitch, continuous_plan));
      data["plan"].update({{"yaw_vel", plan.yaw_vel}, {"pitch_vel", plan.pitch_vel},
                           {"yaw_acc", plan.yaw_acc}, {"pitch_acc", plan.pitch_acc}});
      const double yaw_error = tools::limit_rad(plan.target_yaw - gun_yaw);
      const double pitch_error = plan.target_pitch - gun_pitch;
      data["error"].update({{"yaw", yaw_error}, {"pitch", pitch_error},
        {"yaw_deg", yaw_error * 180 / CV_PI}, {"pitch_deg", pitch_error * 180 / CV_PI}});
    } else {
      target_yaw_curve.reset();
    }
    if (!targets.empty()) {
      const auto x = targets.front().ekf_x();
      const double continuous_phase = phase_curve.update(x[6], x[6]);
      data["target"].update({{"phase", x[6]}, {"phase_deg", x[6] * 180 / CV_PI},
        {"phase_unwrapped", continuous_phase}, {"phase_unwrapped_deg", continuous_phase * 180 / CV_PI},
        {"phase_rate", x[7]}});
    } else {
      phase_curve.reset(); last_target_updates = -1;
    }
    const auto end = Clock::now();
    const double processing_ms = milliseconds(end - process_start);
    data.update({{"processing_ms", processing_ms}, {"loop_ms", milliseconds(end - begin)},
      {"receive_to_process_ms", std::max(0.0, milliseconds(process_start.time_since_epoch()) - frame.received_monotonic_ns / 1e6)},
      {"command_yaw", plan.yaw}, {"command_pitch", plan.pitch},
      {"prediction_horizon_ms", plan.prediction_horizon * 1000}});
    if (transport) {
      const auto stats = transport->stats();
      data.update({{"received", stats.received}, {"replaced", stats.replaced}, {"mismatch", stats.mismatch},
                   {"reconnects", stats.reconnects}, {"clock_resets", stats.clock_resets}});
    }
    ages.add(age_ms);
    processing_times.add(processing_ms);
    if (end - report_time >= 1s) {
      age_p50 = ages.percentile(0.50);
      age_p95 = ages.percentile(0.95);
      age_p99 = ages.percentile(0.99);
      processing_p95 = processing_times.percentile(0.95);
    }
    data.update({{"frame_age_p50_ms", age_p50}, {"frame_age_p95_ms", age_p95},
      {"frame_age_p99_ms", age_p99}, {"processing_p95_ms", processing_p95}});
    ++frames;
    age_sum += age_ms;
    processing_sum += processing_ms;
    data["fps"] = frames / std::max(0.001, std::chrono::duration<double>(end - start).count());
    if (plotter && end >= next_plot) {
      // A small numeric-only JSON packet; the nonblocking UDP sender never
      // waits for a viewer or serializes the image/EKF matrices.
      const nlohmann::json curves = {{"timestamp", frame.timestamp_ns / 1e9}, {"time", data["time"]},
        {"frame_seq", frame.sequence}, {"target", data["target"]}, {"gimbal", data["gimbal"]},
        {"plan", data["plan"]}, {"error", data["error"]}, {"fire", send && firing.fire},
        {"tracking", !targets.empty()}, {"converged", data["converged"]}};
      plotter->plot(curves);
      next_plot = end + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / plot_rate));
    }
    if (debug) debug->publish(std::move(visualization));
    if (end - report_time >= 1s) {
      tools::logger()->info("frames={} detected={} tracking={} age={:.1f}ms p95={:.1f}ms process={:.1f}ms stale={} fire={} shots={} launched={} hits={}",
        frames, detected, !targets.empty(), age_ms, age_p95, processing_ms, stale,
        auto_aim::to_string(firing.reason), shots_requested, frame.launch_count, frame.hit_count);
      report_time = end;
    }
  }
  tools::logger()->info("Finished: {} frames, mean age {:.2f}ms, mean processing {:.2f}ms", frames,
                        frames ? age_sum / frames : 0, frames ? processing_sum / frames : 0);
  return 0;
} catch (const std::exception &error) {
  std::cerr << "task1_sim: " << error.what() << '\n';
  return 1;
}
