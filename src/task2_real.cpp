#include <chrono>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/yolo.hpp"
#include "task/auto_aim/tracker.hpp"
#include "task/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys = "{help h usage ? | | 输出命令行参数说明}"
                         "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  // Khởi tạo bộ phân tích cú pháp dòng lệnh của OpenCV
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path"))
  {
    cli.printMessage();
    return 0;
  }

  // 1. Khởi tạo các doi tuong
  tools::Exiter exiter; // bắt tín hiệu Ctrl+C (SIGINT) giúp thoát chương trình an toàn
  tools::Plotter plotter;
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path);
  auto_aim::Planner planner(config_path);

  // 2. Đọc config
  auto config = YAML::LoadFile(config_path);
  double bullet_speed = config["bullet_speed"] ? config["bullet_speed"].as<double>() : 25.0;
  double execution_delay_s = config["execution_delay_ms"] ? config["execution_delay_ms"].as<double>() / 1000.0 : 0.02;

  Eigen::Vector3d t_gimbal_muzzle(0, 0, 0);
  if (config["t_gimbal_muzzle"])
  {
    auto arr = config["t_gimbal_muzzle"].as<std::vector<double>>();
    t_gimbal_muzzle = Eigen::Vector3d(arr[0], arr[1], arr[2]);
  }

  // 3. Khởi tạo file log
  std::ofstream csv_log("/tmp/real_exp_log.csv");
  csv_log << "time,target_yaw,plan_yaw,gimbal_yaw,obs_yaw,est_vx,est_vy,est_vyaw\n";
  csv_log << std::fixed << std::setprecision(6);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;
  auto last_plot_time = std::chrono::steady_clock::now();

  // 4. Vòng lặp chính
  while (!exiter.exit())
  {
    try
    {
      // A. Đọc ảnh
      camera.read(img, t);

      // B. Lấy góc Gimbal
      Eigen::Quaterniond q_world_gimbal = gimbal.q(t); // Hàm q(t) nội suy ra góc quay dưới dạng Quaternion (4 chiều) thay vì góc Euler (3 chiều) để tránh Gimbal Lock

      // C. YOLO
      auto armors = yolo.detect(img, frame_count++); // yolo.detect() đưa ma trận ảnh qua mạng nơ-ron, trả về danh sách Armor (chứa tọa độ 4 góc pixel)
      if (armors.empty())
      {
        gimbal.send(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // Nếu danh sách rỗng (mất dấu), lập tức gửi tín hiệu gimbal.send(false) để nhả quyền điều khiển động cơ, đảm bảo súng đứng yên an toàn, sau đó bỏ qua frame này (continue)
        continue;
      }

      // D. Solver
      Eigen::Vector3d p_world_camera = Eigen::Vector3d::Zero();
      solver.set_world_camera_pose(q_world_gimbal, p_world_camera); // Đồng bộ hóa hệ tọa độ Camera với Gimbal

      // E. PnP
      std::list<auto_aim::Armor> observations;
      for (auto &armor : armors)
      {
        if (solver.try_solve(armor)) // Hàm try_solve() thực thi thuật toán PnP, sử dụng 4 điểm ảnh 2D và ma trận nội tại -> tìm ra tọa độ 3D (x,y,z) và góc xoay (yaw) của bảng giáp
        {
          observations.push_back(armor); // bảng giáp giải thành công được đưa vào danh sách observations
        }
      }
      if (observations.empty())
      {
        gimbal.send(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        continue;
      }

      // F. Tracker
      auto target_opt = tracker.track(observations, t); // tracker.track() chuyển tọa độ PnP (rất nhiễu) vào bộ lọc EKF (Loại bỏ outlier (nhiễu); Triệt tiêu rung lắc; Ước lượng (vx, vy) mà PnP không thể đo trực tiếp)
      if (!target_opt.has_value())                      // ko co gia tri -> mat dau hoan toan

      {
        gimbal.send(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        continue;
      }

      auto &target = target_opt.value();

      // Lấy vận tốc từ Target
      double est_vx = 0, est_vy = 0, est_vyaw = 0;
      if (target.is_initialized())
      {
        Eigen::VectorXd x = target.get_state(); // Trích xuất Vector trạng thái 8 chiều của EKF
        est_vx = x(3);                          // chuẩn bị xuất ra PlotJuggler làm bằng chứng vx vy vyaw
        est_vy = x(4);
        est_vyaw = x(7);
      }

      // G. Muzzle & Frame Age
      Eigen::Vector3d p_world_gimbal = Eigen::Vector3d::Zero();
      Eigen::Vector3d muzzle_world = p_world_gimbal + q_world_gimbal * t_gimbal_muzzle;
      auto now = std::chrono::steady_clock::now();
      double age_s = std::chrono::duration<double>(now - t).count();

      // H. Planner - TÍNH TOÁN ĐƯỜNG BẮN ĐÓN
      auto_aim::PlanningContext context;
      context.observation_time = t;
      context.muzzle_world = muzzle_world;
      context.prediction_delay_s = age_s + execution_delay_s;
      auto plan = planner.plan(target, bullet_speed, context);

      // I. Gửi lệnh
      if (plan.control)
      {
        gimbal.send(true, plan.fire,
                    plan.yaw, plan.yaw_vel, plan.yaw_acc,
                    plan.pitch, plan.pitch_vel, plan.pitch_acc);
      }
      else
      {
        gimbal.send(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      }

      // J. Plotter & CSV
      double plot_dt = std::chrono::duration<double>(now - last_plot_time).count();
      if (plot_dt > 0.01)

      {
        last_plot_time = now;

        // Lấy yaw thô từ camera (an toàn)
        double obs_yaw = 0.0;
        if (!observations.empty())
        {
          obs_yaw = observations.front().ypr_in_world[0];
        }

        // Lấy state từ EKF
        double target_yaw_est = 0, est_vx = 0, est_vy = 0, est_vyaw = 0;
        if (target.is_initialized())
        {
          Eigen::VectorXd x = target.get_state();
          target_yaw_est = x(6); // Góc yaw EKF ước lượng
          est_vx = x(3);
          est_vy = x(4);
          est_vyaw = x(7);
        }

        nlohmann::json j;
        j["time"] = std::chrono::duration<double>(now.time_since_epoch()).count();
        j["plan"]["yaw"] = plan.yaw;
        j["plan"]["pitch"] = plan.pitch;
        j["plan"]["valid"] = plan.control;
        j["plan"]["distance"] = plan.distance;
        j["plan"]["error"] = plan.tracking_error;

        // Góc EKF ước lượng (hiện tại)
        j["target"]["yaw"] = target_yaw_est;
        // Góc Planner muốn bắn (tương lai, có thể bị offset)
        j["target"]["yaw_plan"] = plan.target_yaw;

        Eigen::Vector3d euler = q_world_gimbal.toRotationMatrix().eulerAngles(2, 1, 0);
        j["gimbal"]["yaw"] = euler[0];
        j["gimbal"]["pitch"] = euler[1];

        j["est"]["vx"] = est_vx;
        j["est"]["vy"] = est_vy;
        j["est"]["vyaw"] = est_vyaw;
        j["obs"]["yaw"] = obs_yaw;

        plotter.plot(j);

        // Ghi CSV
        csv_log << j["time"].get<double>() << ","
                << j["target"]["yaw"].get<double>() << "," // Góc EKF
                << j["plan"]["yaw"].get<double>() << ","
                << j["gimbal"]["yaw"].get<double>() << ","
                << j["obs"]["yaw"].get<double>() << ","
                << j["est"]["vx"].get<double>() << ","
                << j["est"]["vy"].get<double>() << ","
                << j["est"]["vyaw"].get<double>() << "\n";
        csv_log.flush();
      }
    } 
    /*/ [DEBUG] VẼ HÌNH TRÒN VÀ CROSSHAIR
    // ==========================================
    if (!img.empty())
    {
      // 1. Vẽ hình tròn tại tâm bảng giáp (Tọa độ PnP)
      for (const auto &obs : observations)
      {
        // Lấy tọa độ pixel của tâm bảng giáp
        cv::Point2f center(obs.xy_in_pixel.x, obs.xy_in_pixel.y);

        // Vẽ hình tròn màu xanh lá (bán kính 20 pixel, độ dày nét 3)
        cv::circle(img, center, 20, cv::Scalar(0, 255, 0), 3);

        // Vẽ dấu thập nhỏ ở giữa
        cv::line(img, center + cv::Point2f(-10, 0), center + cv::Point2f(10, 0), cv::Scalar(0, 255, 0), 2);
        cv::line(img, center + cv::Point2f(0, -10), center + cv::Point2f(0, 10), cv::Scalar(0, 255, 0), 2);
      }

      // 2. Vẽ Crosshair (Tâm ngắm) ở chính giữa màn hình
      // Đây là điểm mà Gimbal đang thực sự chĩa vào
      cv::Point2f screen_center(img.cols / 2.0, img.rows / 2.0);
      cv::circle(img, screen_center, 100, cv::Scalar(0, 0, 255), 1); // Vòng tròn đỏ to
      cv::line(img, screen_center + cv::Point2f(-50, 0), screen_center + cv::Point2f(50, 0), cv::Scalar(0, 0, 255), 2);
      cv::line(img, screen_center + cv::Point2f(0, -50), screen_center + cv::Point2f(0, 50), cv::Scalar(0, 0, 255), 2);

      // 3. In thông số lỗi
      std::string text1 = (target.is_initialized() ? "TRACKING" : "LOST");
      cv::putText(img, text1, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

      cv::imshow("Camera Debug", img);
      cv::waitKey(1); // Cực kỳ quan trọng để hình hiển thị
    }
    // ==========================================*/
    catch (const std::exception &error)
    {
      std::cerr << "task2_real (frame error): " << error.what() << "\n";
    }
  }

  return 0;
}