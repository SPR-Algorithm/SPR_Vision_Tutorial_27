#include <chrono>
#include <exception>
#include <iostream>
#include <list>
#include <memory>

#include <nlohmann/json.hpp>

#include "sim_bridge/foxglove_debug.hpp"
#include "sim_bridge/simulation_config.hpp"
#include "sim_bridge/simulator.hpp"
#include "task/auto_aim/planner/planner.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/tracker.hpp"
#include "task/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) try {
  // 解析命令行 + YAML 配置，得到一堆运行参数（帧龄上限、弹速、时长……）。
  const auto options = sim_bridge::load_sim_options(argc, argv);
  const auto &config_path = options.config;

  // ---- 初始化调试工具类 ----
  tools::Exiter exiter; // 捕获 Ctrl+C，用来安全退出主循环
  std::unique_ptr<tools::Plotter> plotter;
  if (options.plotjuggler_enabled) {
    plotter = std::make_unique<tools::Plotter>(options.plot_host, options.plot_port);
  }

  // ---- 初始化仿真 IO 类 ----
  // Simulator：负责从模拟器的共享内存里"读帧"，以及向模拟器"发云台指令"。
  sim_bridge::Simulator simulator(options.shm_directory);
  std::unique_ptr<sim_bridge::FoxgloveDebug> foxglove;
  if (options.foxglove_enabled) {
    foxglove = std::make_unique<sim_bridge::FoxgloveDebug>(
      options.foxglove_host, static_cast<std::uint16_t>(options.port),
      options.foxglove_rate, options.foxglove_visual_rate);
  }

  // ---- 初始化 auto_aim 各组件 ----
  auto_aim::YOLO yolo(config_path, true); // 检测器（第2步）
  auto_aim::Solver solver(config_path);   // 解算器 PnP（第3步）
  auto_aim::Tracker tracker(config_path); // 目标管理（第4步）
  auto_aim::Planner planner(config_path); // 规划器（第6步）

  sim_bridge::SimFrame frame; // 一帧数据：图像 + 位姿 + 时间戳

  // 记录循环开始时刻，用来实现 --duration 限时。
  const auto start_time = std::chrono::steady_clock::now();

  // 帧计数器，方便日志里看处理了多少帧。
  std::uint64_t frame_count = 0;

  while (!exiter.exit()) {
    // ============================================================
    // 第1步：读一帧（图像 + 相机/云台位姿 + 时间戳）
    // ============================================================
    if (!simulator.read(frame, std::chrono::milliseconds(100))) {
      continue; // 100ms 内没等到新帧，直接回头再等（不让主循环空转卡死）
    }
    frame_count++;

    // --duration 限时退出：跑够时长就自己停。
    if (options.duration > 0) {
      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      if (elapsed >= options.duration) break;
    }

    // 帧龄限制：太旧的帧不要用（说明处理/传输延迟太大，用旧数据会瞄不准）。
    if (sim_bridge::capture_age_ms(frame) > options.max_age_ms) {
      continue;
    }

    // ============================================================
    // 第2步：检测——在图像里找出装甲板（结果还是像素坐标）
    // ============================================================
    auto armors = yolo.detect(frame.bgr, static_cast<int>(frame.sequence));

    // ============================================================
    // 第3步：解算——用 PnP 把像素坐标换算成世界坐标（单位米）
    // ============================================================
    // 每一帧都要把"这一帧的相机标定 + 相机世界位姿"喂给 Solver。
    solver.set_camera_calibration(frame.camera_matrix(), frame.distortion_coeffs());
    solver.set_world_camera_pose(frame.world_camera_rotation(), frame.world_camera_position());

    std::list<auto_aim::Armor> solved; // 只保留解算成功的观测
    for (auto &armor : armors) {
      if (solver.try_solve(armor)) solved.push_back(std::move(armor));
    }

    // ============================================================
    // 第4步：跟踪——把这一帧的观测喂给 Tracker（建立/关联/失锁）
    // ============================================================
    auto target = tracker.track(solved, frame.timestamp);
    if (!target) {
      // 没有可用的目标，跳过这一帧（没法规划、也没法瞄准）。
      continue;
    }

    // ============================================================
    // 第5步 + 第6步：预测 + 规划
    // ============================================================
    // PlanningContext 是规划器要的"上下文"，告诉它三个关键信息：
    auto_aim::PlanningContext ctx;
    ctx.observation_time = frame.timestamp; // 这帧观测的采集时刻
    // 枪口在世界系下的位置 = 云台位置 + 云台姿态旋转后的枪口偏移量
    ctx.muzzle_world = frame.p_world_gimbal + frame.q_world_gimbal * frame.t_gimbal_muzzle;
    // 从"采集"到"指令真正执行"的延迟 = 帧龄 + 执行延迟（不包含弹丸飞行，规划器自己会加）
    ctx.prediction_delay_s = sim_bridge::capture_age_ms(frame) / 1000.0 + options.delay_s;

    // 规划器会多次调用 target.predict() 问"未来某时刻目标在哪"，再解出云台角度。
    auto plan = planner.plan(*target, options.bullet_speed, ctx);

    // ============================================================
    // 第7步：发指令给云台
    // ============================================================
    if (plan.control) {
      // plan.yaw / plan.pitch 是算出来的云台角度（弧度）。
      // 第三个参数 fire=false：跟随阶段不开火。
      simulator.send(plan.yaw, plan.pitch, false, plan.distance);
    }

    // ============================================================
    // 调试输出（给 PlotJuggler / Foxglove 看的，方便出报告画曲线）
    // ============================================================
    // —— PlotJuggler：发一条 UDP 曲线数据 ——
    if (plotter) {
      nlohmann::json j;
      j["timestamp"] = static_cast<double>(frame.timestamp_ns) / 1e9; // 采集时刻（UNIX 秒）
      j["valid"] = plan.control;          // 本帧是否有效
      j["target_yaw"] = plan.target_yaw;  // 目标方向（来自预测）
      j["target_pitch"] = plan.target_pitch;
      j["plan_yaw"] = plan.yaw;           // 规划出的指令方向
      j["plan_pitch"] = plan.pitch;
      j["distance"] = plan.distance;
      plotter->plot(j);
    }

    // —— Foxglove：发场景（3D 点）和指标 ——
    if (foxglove) {
      sim_bridge::DebugFrame dbg;
      dbg.timestamp_ns = frame.timestamp_ns;
      dbg.has_target = true;

      // measured：这一帧解算出的原始观测世界坐标（还没滤波的"原始值"）
      for (const auto &a : solved) dbg.measured.push_back(a.xyz_in_world);

      // predicted：滤波 + 预测之后的位置（平滑值）
      auto pred = target->predict(frame.timestamp);
      for (const auto &p : pred.armors) dbg.predicted.push_back(p.xyza.head<3>());

      dbg.telemetry["valid"] = plan.control;
      dbg.telemetry["target_yaw"] = plan.target_yaw;
      dbg.telemetry["target_pitch"] = plan.target_pitch;
      dbg.telemetry["plan_yaw"] = plan.yaw;
      dbg.telemetry["plan_pitch"] = plan.pitch;
      dbg.telemetry["distance"] = plan.distance;

      foxglove->publish(dbg);
    }
  }

  tools::logger()->info("task1_sim 正常退出，共处理 {} 帧", frame_count);
  return 0;
} catch (const std::exception &error) {
  std::cerr << "task1_sim: " << error.what() << '\n';
  return 1;
}
