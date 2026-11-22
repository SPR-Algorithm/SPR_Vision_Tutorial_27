#include <chrono>
#include <exception>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "task/auto_aim/planner/planner.hpp"
#include "task/auto_aim/shooter.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/tracker.hpp"
#include "task/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys = "{help h usage ? | | 输出命令行参数说明}"
                         "{@confiq'qg-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char *argv[]) try {
  // ---- 解析命令行：拿到 yaml 配置路径 ----
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // ---- 初始化工具类 ----
  tools::Exiter exiter;   // 捕获 Ctrl+C，用来安全退出主循环
  tools::Plotter plotter; // 调试曲线，UDP 发给本机 PlotJuggler（发不出去也不阻塞主循环）

  // ---- 初始化 io 类：相机取图 + 云台串口收发 ----
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // ---- 初始化 auto_aim 各组件（和仿真 task1_sim 是同一套）----
  auto_aim::YOLO yolo(config_path, true); // 检测器（第2步）
  auto_aim::Solver solver(config_path);   // 解算器 PnP（第3步）
  auto_aim::Tracker tracker(config_path); // 目标管理 + 卡尔曼滤波（第4步）
  auto_aim::Planner planner(config_path); // 规划器：预测位置 -> 云台角度（第6步）
  auto_aim::Shooter shooter(config_path); // 发射判定（第8步）

  // ---- 兜底弹速 ----
  // 真实弹速由电控通过串口下发（gimbal.state().bullet_speed）。
  // 万一电控还没下发（值为 0），用配置里的 bullet_speed 兜底，保证中期考核能跑。
  // 配置里没写就用 24.9 m/s（17mm 弹丸常见初速）。
  double fallback_bullet_speed = 24.9;
  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["bullet_speed"]) fallback_bullet_speed = yaml["bullet_speed"].as<double>();
  } catch (const std::exception & e) {
    tools::logger()->warn("读取兜底弹速失败，用默认 24.9 m/s：{}", e.what());
  }

  // 图像 + 采集时刻（注意：t 是"采集"时刻，不是处理完成时刻，滤波/预测都要用它）
  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  // 帧计数器（给 YOLO 做调试打印用）
  std::uint64_t frame_count = 0;

  // 连续跟踪帧数，用来判断"滤波是否已收敛"（简单启发式）。
  int converged_frames = 0;

  // 串口下发 + 云台电机响应的额外延迟（秒）。这是估计值，可根据实车表现微调。
  constexpr double kExecutionDelay = 0.05;

  // 给 Plotter 提供"墙上时钟"（UNIX 秒）时间轴：启动时记下 steady 和 system 的对应关系。
  const auto steady_start = std::chrono::steady_clock::now();
  const auto wall_start = std::chrono::system_clock::now();

  while (!exiter.exit()) {
    // ============================================================
    // 第1步：读一帧（图像 + 采集时刻）
    // ============================================================
    camera.read(img, t);
    if (img.empty()) continue; // 相机还没出图 / 掉线，回头再等，不让主循环空转崩溃
    frame_count++;

    // ============================================================
    // 第2步：检测——在图像里找装甲板（结果还是像素坐标）
    // ============================================================
    auto armors = yolo.detect(img, static_cast<int>(frame_count));

    // ============================================================
    // 第3步：解算——PnP 把像素坐标换算成世界坐标（单位：米）
    // ============================================================
    // 相机和云台固连。相机标定 + 相机相对云台的外参（R_camera2gimbal/t_camera2gimbal）
    // 都在 config 里、Solver 构造时已经读好了。这里每帧只需要更新"云台在世界系下的姿态"。
    solver.set_R_gimbal2world(gimbal.q(t));

    std::list<auto_aim::Armor> solved; // 只保留解算成功的观测
    for (auto & armor : armors) {
      if (solver.try_solve(armor)) solved.push_back(std::move(armor));
    }

    // ============================================================
    // 第4步：跟踪——把这一帧观测喂给 Tracker（建立/关联/失锁）
    // ============================================================
    auto target = tracker.track(solved, t);
    if (!target) {
      // 没有目标：发一条"不控制"指令，让云台停住（或保持），然后处理下一帧。
      gimbal.send(false, false, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
      shooter.reset();
      converged_frames = 0;
      continue;
    }

    // ============================================================
    // 第5步：准备规划上下文
    // ============================================================
    auto_aim::PlanningContext ctx;
    ctx.observation_time = t; // 采集时刻：规划器从这一刻开始问"未来目标在哪"

    // 枪口世界坐标。世界系原点就在云台旋转中心（Solver 里 t_gimbal2world = 0），
    // 枪口相对云台中心的偏移量没有标定数据，这里近似取云台中心（原点）。
    // 这点偏移对"跟随"的瞄准角度影响极小；以后要做精确弹道再补枪口偏移。
    ctx.muzzle_world = Eigen::Vector3d::Zero();

    // 预测延迟 = 从采集到现在已经花掉的处理时间 + 串口/云台执行延迟。
    const double age_s = tools::delta_time(std::chrono::steady_clock::now(), t);
    ctx.prediction_delay_s = age_s + kExecutionDelay;

    // 弹速：优先用电控下发的真实值；没下发（<=1）就用兜底值。
    double bullet_speed = gimbal.state().bullet_speed;
    if (!(bullet_speed > 1.0)) bullet_speed = fallback_bullet_speed;

    // ============================================================
    // 第6步：规划——把"未来目标位置"解成云台 yaw/pitch 指令
    // ============================================================
    // planner 会多次调用 target.predict() 问未来位置，再结合弹道/MPC 解出角度。
    auto plan = planner.plan(*target, bullet_speed, ctx);

    // ============================================================
    // 第7步：发射判定 + 发指令给云台
    // ============================================================
    // 收敛判定：连续跟踪若干帧后认为滤波已收敛（简单启发式，够用）。
    if (tracker.state() == "tracking") {
      converged_frames++;
    } else if (tracker.state() == "lost") {
      converged_frames = 0;
    }
    const bool converged = converged_frames >= 15;

    // 把"这一帧的规划 + 跟踪状态 + 云台实际角度"喂给 Shooter，
    // 由它决定这一帧到底开不开火（瞄准对齐、冷却、稳定等条件它都会检查）。
    auto_aim::FireInput fire_in;
    fire_in.plan = plan;
    fire_in.tracking = true;                              // 有目标（走到这说明 target 存在）
    fire_in.converged = converged;                        // 滤波是否已收敛
    fire_in.measurement = (tracker.state() == "tracking"); // 这一帧有没有新的观测
    fire_in.following = plan.control;                     // 是否正在控制云台跟随
    fire_in.control_enabled = true;                       // 云台控制使能
    fire_in.fire_enabled = true;                          // 发射使能（运行开关）
    fire_in.frame_age_s = age_s;                          // 帧龄（秒）
    fire_in.gun_yaw = gimbal.state().yaw;                 // 云台实际 yaw
    fire_in.gun_pitch = gimbal.state().pitch;             // 云台实际 pitch
    fire_in.now = std::chrono::steady_clock::now();

    const auto decision = shooter.update(fire_in);

    if (plan.control) {
      // fire 由 Shooter 判定：瞄准对齐且稳定后才开火。
      gimbal.send(true, decision.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc,
                  plan.pitch, plan.pitch_vel, plan.pitch_acc);
    } else {
      // 有目标但规划失败（弹道无解等），停住云台、不开火。
      gimbal.send(false, false, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    }

    // ============================================================
    // 调试输出（发给 PlotJuggler 画曲线，看"云台有没有跟上目标"）
    // ============================================================
    const double wall_s = std::chrono::duration<double>(wall_start.time_since_epoch()).count() +
                          tools::delta_time(std::chrono::steady_clock::now(), steady_start);
    nlohmann::json j;
    j["timestamp"] = wall_s;          // 墙上时钟秒，PlotJuggler 当时间轴
    j["valid"] = plan.control;        // 这一帧有没有有效指令
    j["fire"] = decision.fire;        // 这一帧是否开火
    j["state"] = tracker.state();     // 跟踪状态：lost / tracking / temporary_lost
    j["bullet_speed"] = bullet_speed; // 当前用的弹速（能看出电控有没有下发）
    j["target_yaw"] = plan.target_yaw;   // 目标方向（来自预测）
    j["target_pitch"] = plan.target_pitch;
    j["plan_yaw"] = plan.yaw;            // 规划出的云台指令
    j["plan_pitch"] = plan.pitch;
    j["distance"] = plan.distance;
    plotter.plot(j);
  }

  tools::logger()->info("task2_real 正常退出，共处理 {} 帧", frame_count);
  return 0;
} catch (const std::exception & error) {
  std::cerr << "task2_real: " << error.what() << '\n';
  return 1;
}
