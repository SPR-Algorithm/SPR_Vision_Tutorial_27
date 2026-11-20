#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
using TimePoint = std::chrono::steady_clock::time_point;

// 控制与可视化使用的输出格式，不规定滤波器的内部状态。
struct PredictedArmor
{
  // 与输入观测相同的世界坐标系，位置单位 m。
  // 朝向沿用 Solver 写入的 Armor::ypr_in_world[0] 约定，单位 rad。
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
  // 维护的非负关联编号，默认 -1 表示无效；不是模拟器真值编号。
  int id = -1;
};

struct Prediction
{
  bool valid = false;
  std::vector<PredictedArmor> armors;
};

// enum class TargetModel
// {
//   none,
//   translation,
//   outpost
// };

class Target
{
public:
  explicit Target(const std::string & config_path);

  void reset();

  // observations 应先由调用者通过 Solver 解算到同一世界坐标系。
  // timestamp 是这帧观测的采集时刻，不能以处理完成时刻替代。
  void update(const std::vector<Armor> & observations, TimePoint timestamp);

  // 查询指定未来时刻的预测结果，不改变当前滤波状态。
  Prediction predict(TimePoint query_time) const;

  bool initialized() const { return initialized_; }
  bool isOutpost() const { return outpost_; }

private: 
  void loadConfig();
  void initTranslation(const Armor & seed, TimePoint t);
  void predictStep(double dt);
  void initOutpost(const Armor & seed, TimePoint t);
  void updateTranslation(const std::vector<Armor> & obs);
  void updateOutpost(const std::vector<Armor> & obs);

  std::string config_path_;
  tools::ExtendedKalmanFilter ekf_;
  bool initialized_ = false;
  bool outpost_ = false;
  TimePoint timestamp_{};
  double last_yaw_ = 0.0;


    //平移模型参数
  bool is_outpost_ = false;                 // 强制前哨站模型，否则按观测名自动判断
  double q_trans_ = 1.0;                    // 速度随机游走 PSD (m/s)^2/s
  Eigen::Vector3d r_diag_ = Eigen::Vector3d(0.0009, 0.0009, 0.0025);  // R 对角
  Eigen::Vector3d p0_pos_ = Eigen::Vector3d(0.0025, 0.0025, 0.0025);  // 初始位置方差
  double p0_vel_ = 1.0;                     // 初始速度方差
  double max_dt_ = 0.5;                     // 帧间隔阈值（超过则重建）
  double max_horizon_ = 1.0;                // 预测最远时刻

 //前哨站模型参数
  double q_alpha_ = 30.0;   // 角加速度 PSD rad^2/s^3
  double r_yaw_ = 0.08;     // 朝向观测噪声 rad
  double radius_ = 0.275;   // 板到旋转轴距离 m（Ø550 → 275 mm）
  std::vector<double> heights_ = {-0.102, 0.0, 0.102};  // 相对旋转中心的高度 m
  std::vector<double> angles_ = {0.0, 2.094395102, 4.188790205};  // rad（0/120/240°）
  double center_z_prior_ = 1.216;  // 初始化选槽用的安装高度 m（图纸 1110/1146 mm）
  
  // TODO(学生): 按自己的建模方案补充内部数据与方法。
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
