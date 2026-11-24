#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"
#include <stdexcept>
#include "task/auto_aim/planner/planner.hpp"   
#include "task/auto_aim/tracker.hpp"        

const std::string keys = "{help h usage ? | | 输出命令行参数说明}"
                         "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char *argv[]) try{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  //参数
  const auto yaml = tools::load(config_path);
  const bool control_enabled = tools::read<bool>(yaml, "control", true);
  const bool fire_enabled = tools::read<bool>(yaml, "fire", true);
  const double bullet_speed_fallback = tools::read<double>(yaml, "bullet_speed", 25.0);
  const double execution_delay_s = tools::read<double>(yaml, "execution_delay_ms", 0.0) / 1000.0;
  const auto muzzle_off =
    tools::read<std::vector<double>>(yaml, "t_gimbal_muzzle", {0.0, 0.0, 0.0});
  if (muzzle_off.size() != 3) throw std::invalid_argument("t_gimbal_muzzle must have 3 entries");
  const Eigen::Vector3d t_muzzle(muzzle_off[0], muzzle_off[1], muzzle_off[2]);

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter; // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path);
  auto_aim::Planner planner(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  int frame_index = 0;
  bool last_fire =false;
  
  while (!exiter.exit()) {
    // Your code start

    //取图
    camera.read(img, t);
    if (cv::waitKey(1) == 'q') {
      break;
    }
    
    if (img.empty()){
      std::cout<<"empty";
      continue;} //空帧检测

    //云台姿态
    q = gimbal.q(t); //云台读取到的IMU四元数
    const auto gs = gimbal.state();
    const auto mode = gimbal.mode();
    solver.set_R_gimbal2world(q); //将云台坐标转化为世界坐标
    const Eigen::Vector3d gimbal_ypr = tools::eulers(q, 2, 1, 0); //解算云台坐标为欧拉角

    //检测,PnP
    auto armors = yolo.detect(img,frame_index++); //通过yolo来识别装甲板
    std::list<auto_aim::Armor> solved; //存放解算成功的装甲板
    for (auto & a : armors) {
      if (solver.try_solve(a)) solved.push_back(a); //尝试解算，成功的放进a中
    }
 
    //目标管理,滤波
    auto target = tracker.track(solved, t); //目标检测
    const auto now = std::chrono::steady_clock::now(); //取解算成功时间
    const double age_s = std::max(0.0, tools::delta_time(now, t)); //解算帧龄
    const double bullet_speed = gs.bullet_speed > 1.0 ? gs.bullet_speed : bullet_speed_fallback; //检测弹丸的发射速度，若小于1则无效，拿25来算

    //打包
    auto_aim::Plan plan; //规划
    if (target) {
      auto_aim::PlanningContext ctx; //采集时刻、枪口世界位置、延迟
      ctx.observation_time = t; //采集时刻
      ctx.muzzle_world = solver.R_gimbal2world() * t_muzzle; //枪口的世界位置
      ctx.prediction_delay_s = age_s + execution_delay_s; //延迟
      plan = planner.plan(*target, bullet_speed, ctx);
    }

    //下发
    const bool fresh = age_s < 0.1; //新鲜度
    const bool tracking =(tracker.state()=="tracking"); //是否锁到目标
    const bool coasting =(tracker.state()=="temporarily_lost"); //是否超时丢目标
    const bool trackering_state= tracking || coasting ; //锁定状态
    const bool ctrl = control_enabled && mode == io::GimbalMode::AUTO_AIM && plan.control && trackering_state && fresh; //是否可以控制
    const bool fire = (tracking && plan.fire && fire_enabled); //是否建议开火
    
    if (ctrl) {
      gimbal.send(true, fire, plan.yaw, plan.yaw_vel, plan.yaw_acc,-plan.pitch, -plan.pitch_vel, -plan.pitch_acc); //发送预测后的云台角度
                  //gimbal.send(true, false, 0.3F, 0.0F, 0.0F, gs.pitch, 0.0F, 0.0F);
                  //gimbal.send(true, false, gs.yaw, 0.0F, 0.0F, 0.3F, 0.0F, 0.0F);
    } else {
      gimbal.send(false, false, gs.yaw, gs.yaw_vel, 0.0F, gs.pitch, gs.pitch_vel, 0.0F); //保持原有的云台角度不变
    }

    if (fire != last_fire) {
      tools::logger()->info(
        "[Fire] fire={} plan.fire={} err={:.5f} (thr=0.0035) aim_id={} state={}",
        fire, plan.fire, plan.tracking_error, plan.aim_id, tracker.state());
      last_fire = fire;
    }

    // ---- 调试曲线 ----
    nlohmann::json m;
    m["gimbal_yaw"] = gs.yaw;
    m["gimbal_pitch"] = gs.pitch;
    m["imu_yaw"] = gimbal_ypr[0];
    m["plan_yaw"] = plan.control ? plan.yaw : 0.0;
    m["plan_pitch"] = plan.control ? plan.pitch : 0.0;
    m["tracking"] = target.has_value();
    m["state"] = tracker.state();
    m["age_ms"] = age_s * 1e3;
    plotter.plot(m);
    // Your code end
  }
  return 0;
  
} catch (const std::exception & e) {
  std::cerr << "task2_real: " << e.what() << '\n';
  return 1;
}