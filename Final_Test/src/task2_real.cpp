#include <chrono>
#include <opencv2/opencv.hpp>
#include "sim_bridge/foxglove_debug.hpp"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "task/auto_aim/solver.hpp"
#include "task/auto_aim/yolo.hpp"
#include "task/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"
#include "task/auto_aim/tracker.hpp"

Eigen::MatrixXd vector_to_matrix(const std::vector<double>&v,int rows,int cols)
{
  if(static_cast<size_t>(rows*cols)!=v.size())throw std::invalid_argument("error");
  Eigen::MatrixXd output(rows,cols);
  for(int k=0;k<rows*cols;++k){output(k%cols,k/cols)=v[k];}
  return output;
}
Eigen::VectorXd vector_to_v(const std::vector<double>&v){
  Eigen::VectorXd output=Eigen::Map<const Eigen::VectorXd>(v.data(),v.size());
  return output;
}
const std::string keys = "{help h usage ? | | 输出命令行参数说明}"
                         "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }
  auto yalm=tools::load(config_path);
  // 初始化工具类
  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter>plotter; // 注意plotter工具的使用
  if (tools::read(yalm["plotter"],"enabled",false))
  {
    plotter = std::make_unique<tools::Plotter>(tools::read<std::string>(yalm["plotter"],"host","127.0.0.1"),tools::read<int>(yalm["plotter"],"port",9870));
  }
   std::unique_ptr<sim_bridge::FoxgloveDebug> foxglove;
  if(tools::read(yalm["foxglove"],"enabled",false)){
    foxglove=std::make_unique<sim_bridge::FoxgloveDebug>(tools::read<std::string>(yalm["foxglove"],"host","127.0.0.1"),static_cast<std::uint16_t>(tools::read(yalm["foxglove"],"port",9870)),tools::read(yalm["foxglove"],"rate_hz",100),100);
  }
  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path);
  auto_aim::Planner planner(config_path);
  cv::Mat img;
  sim_bridge::DebugFrame debug_frame;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  Eigen::Matrix3d camera_matrix=vector_to_matrix(tools::read<std::vector<double>>(yalm,"camera_matrix",{1803.85723, 0., 698.29467, 0., 1800.27952, 579.92966, 0., 0., 1.}),3,3);
  Eigen::Vector<double,5>distortion=vector_to_v(tools::read<std::vector<double>>(yalm,"distort_coeffs",{-0.086982, 0.202748, -0.000254, 0.000123, 0.000000}));
  double delay_s=tools::read<double>(yalm,"predict_time",0.120);
  solver.set_camera_calibration(camera_matrix,distortion);
  bool is_fire=tools::read(yalm,"is_fire",false);
  while (!exiter.exit()) {
    // Your code start
    t=std::chrono::steady_clock::now();
    camera.read(img,t);
    if(img.empty())continue;
    solver.set_world_camera_pose(gimbal.q(t),Eigen::Vector3d::Zero());
    auto armors=yolo.detect(img);
    std::list<auto_aim::Armor>solved_armors;
    for(auto&armor:armors){
      if(solver.try_solve(armor)){solved_armors.push_back(armor);}
    }
    auto target_opt = tracker.track(solved_armors,t);
    if (!target_opt.has_value())
      continue;
    auto &target = target_opt.value();
    auto_aim::PlanningContext ctx;
    ctx.observation_time=t;
    ctx.prediction_delay_s=delay_s;
    ctx.muzzle_world=Eigen::Vector3d::Zero();
    auto plan=planner.plan(target,25.0,ctx);
    bool is_control=gimbal.mode()!=io::GimbalMode::IDLE?true:false;
    gimbal.send(is_control?plan.control:false,is_fire?plan.fire:false,plan.yaw,plan.yaw_vel,plan.yaw_acc,plan.pitch,plan.pitch_vel,plan.pitch_acc);
    if (plotter)
    {
      nlohmann::json j;
      j["time"] = std::chrono::duration<double>(t.time_since_epoch()).count();
      if (plan.control)
      {
        if(plan.armor_index>=0&&plan.armor_index<static_cast<int>(solved_armors.size())){
          auto it=std::next(solved_armors.begin(),static_cast<std::ptrdiff_t>(plan.armor_index));
          j["observation.x"]=it->xyz_in_world.x();
          j["observation.y"]=it->xyz_in_world.y();
          j["observation.z"]=it->xyz_in_world.z();
        }
        j["plan.x"]=plan.aim_xyza[0];
        j["plan.y"]=plan.aim_xyza[1];
        j["plan.z"]=plan.aim_xyza[2];
        j["plan.yaw"] = plan.yaw;
        j["plan.pitch"] = plan.pitch;
      }
      plotter->plot(j);
    }
 // 把图像和检测结果发布给Foxglove做3D可视化
    if (foxglove)
    {
      debug_frame.image =img;
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