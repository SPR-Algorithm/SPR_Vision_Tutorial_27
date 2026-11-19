#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "task/auto_aim/debug_utils.hpp"
#include "task/auto_aim/planner/planner.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/tracker.hpp"
#include "task/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys = "{help h usage ? | | 输出命令行参数说明}"
                         "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char *argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path"))
  {
    cli.printMessage();
    return 0;
  }

  // ---- 1. Khởi tạo ----
  tools::Exiter exiter;  // bắt Ctrl+C để thoát an toàn
  tools::Plotter plotter;
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path);
  auto_aim::Planner planner(config_path);

  // ---- 2. Đọc config ----
  const YAML::Node config = YAML::LoadFile(config_path);
  const double bullet_speed = config["bullet_speed"].as<double>(25.0);
  const double execution_delay_s = config["execution_delay_ms"].as<double>(20.0) / 1000.0;
  const bool fire_enabled = config["fire_enabled"].as<bool>(false);  // mặc định KHÔNG bắn khi thử
  Eigen::Vector3d t_gimbal_muzzle(0, 0, 0);
  if (config["t_gimbal_muzzle"])
  {
    const auto arr = config["t_gimbal_muzzle"].as<std::vector<double>>();
    t_gimbal_muzzle = Eigen::Vector3d(arr[0], arr[1], arr[2]);
  }

  // ---- 3. Log CSV (để vẽ báo cáo offline) ----
  std::ofstream csv("/tmp/real_exp_log.csv");
  csv << "time,state,obs_x,obs_y,obs_z,obs_yaw,est_x,est_y,est_z,est_vx,est_vy,est_vz,est_yaw,est_vyaw,nis,"
         "aim_x,aim_y,aim_z,aim_yaw,plan_yaw,plan_pitch,gimbal_yaw,gimbal_pitch,tracking_error\n";
  csv << std::fixed << std::setprecision(6);
  int csv_lines = 0;

  auto send_idle = [&]() { gimbal.send(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); };

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;
  auto last_plot_time = std::chrono::steady_clock::now();

  // ---- 4. Vòng lặp chính ----
  while (!exiter.exit())
  {
    try
    {
      // A. Ảnh + thời điểm chụp (t là timestamp ảnh, dùng xuyên suốt pipeline)
      camera.read(img, t);
      if (img.empty()) continue;

      // B. Tư thế gimbal tại đúng thời điểm chụp (nội suy quaternion)
      const Eigen::Quaterniond q_world_gimbal = gimbal.q(t);

      // C. Detect
      auto armors = yolo.detect(img, frame_count++);

      // D. PnP. Camera đặt ≈ trục gimbal; ngoại tham cam->gimbal do Solver đọc từ YAML.
      solver.set_world_camera_pose(q_world_gimbal, Eigen::Vector3d::Zero());
      std::list<auto_aim::Armor> observations;
      for (auto &armor : armors)
        if (solver.try_solve(armor)) observations.push_back(armor);

      // E. Tracker: LUÔN gọi, kể cả khi rỗng (để bám mù ngắn hạn thay vì nhả gimbal ngay)
      auto target_opt = tracker.track(observations, t);
      if (!target_opt.has_value())
      {
        send_idle();
        continue;
      }
      auto &target = target_opt.value();

      // F. Planner. age_s tính SAU xử lý để bù đúng độ trễ.
      const Eigen::Vector3d muzzle_world = q_world_gimbal * t_gimbal_muzzle;
      const auto now = std::chrono::steady_clock::now();
      auto_aim::PlanningContext context;
      context.observation_time = t;
      context.muzzle_world = muzzle_world;
      context.prediction_delay_s = std::chrono::duration<double>(now - t).count() + execution_delay_s;
      auto plan = planner.plan(target, bullet_speed, context);

      // G. Gửi lệnh. Coasting thì không bắn.
      if (plan.control)
      {
        const bool fire = fire_enabled && plan.fire && !tracker.coasting();
        gimbal.send(true, fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
                    plan.pitch_acc);
      }
      else
      {
        send_idle();
      }

      // H. Plot + CSV (giới hạn 100 Hz)
      if (std::chrono::duration<double>(now - last_plot_time).count() > 0.01)
      {
        last_plot_time = now;
        nlohmann::json j;
        j["time"] = std::chrono::duration<double>(now.time_since_epoch()).count();
        j["tracker"]["state"] = tracker.state();
        j["plan"]["yaw"] = plan.yaw;
        j["plan"]["pitch"] = plan.pitch;
        j["plan"]["valid"] = plan.control;
        j["plan"]["error"] = plan.tracking_error;
        j["target"]["yaw"] = plan.target_yaw;
        j["gimbal"]["yaw"] = auto_aim::debug::gimbal_yaw(q_world_gimbal);
        j["gimbal"]["pitch"] = auto_aim::debug::gimbal_pitch(q_world_gimbal);
        auto_aim::debug::fill_json(j, observations, target, plan.aim_xyza, plan.aim_id);
        plotter.plot(j);

        auto num = [&](const char *a, const char *b, double def = 0.0) {
          return j.contains(a) && j[a].contains(b) ? j[a][b].get<double>() : def;
        };
        csv << j["time"].get<double>() << "," << tracker.state() << "," << num("obs", "x") << ","
            << num("obs", "y") << "," << num("obs", "z") << "," << num("obs", "yaw") << ","
            << num("est", "x") << "," << num("est", "y") << "," << num("est", "z") << ","
            << num("est", "vx") << "," << num("est", "vy") << "," << num("est", "vz") << ","
            << num("est", "yaw") << "," << num("est", "vyaw") << "," << num("est", "nis") << ","
            << num("aim", "x") << "," << num("aim", "y") << "," << num("aim", "z") << ","
            << num("aim", "yaw") << "," << num("plan", "yaw") << "," << num("plan", "pitch") << ","
            << num("gimbal", "yaw") << "," << num("gimbal", "pitch") << "," << num("plan", "error") << "\n";
        if (++csv_lines % 50 == 0) csv.flush();
      }
    }
    catch (const std::exception &error)
    {
      std::cerr << "task2_real (frame error): " << error.what() << "\n";
      send_idle();  // lỗi thì dừng điều khiển, không để lệnh cũ còn hiệu lực
    }
  }

  send_idle();  // thoát (Ctrl+C): nhả gimbal
  csv.flush();
  return 0;
}
