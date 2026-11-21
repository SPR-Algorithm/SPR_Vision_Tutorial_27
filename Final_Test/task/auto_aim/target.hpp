#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>
#include <utility>
#include <optional>
#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/yaml.hpp"
#include "tools/math_tools.hpp"
namespace auto_aim
{
using TimePoint = std::chrono::steady_clock::time_point;

class Tool
{
public:
static double pred_yaw(const std::string&goal,const Eigen::VectorXd&state_,double dt){
  if(goal=="sentry_station"){
    return state_[3]+state_[4]*dt;
  }
 else if(goal=="static_armor"){
   return 0.0;
    }
  return 0.0;
}
static Eigen::Vector3d pred_xyz(const std::string&goal,const Eigen::VectorXd&state_,double angle_,double dh_id,double dt){
  if(goal=="sentry_station"){
  return Eigen::Vector3d(state_[0]+state_[5]*cos(angle_),state_[1]+state_[5]*sin(angle_),state_[2]+state_[6]*dh_id);
  }
 else if(goal=="static_armor"){
    return Eigen::Vector3d(state_[0]+dt*state_[3],state_[1]+dt*state_[4],state_[2]);
  }
  return Eigen::Vector3d::Zero();
}
static void parameter_load(const std::string&goal,const YAML::Node&yalm,double&v1,double&v2,Eigen::MatrixXd&R,Eigen::VectorXd&P0_diag,std::string&color,double&max_distance,double&max_angle_diff,Eigen::VectorXd&X,Eigen::VectorXd&X0,int&N){
   auto yalm_=yalm[goal];
  if(goal=="sentry_station"){
    v1=tools::read<double>(yalm_,"v1",1e-5);v2=tools::read<double>(yalm_,"v2",5e-2);color=tools::read<std::string>(yalm,"enemy_color","none");max_distance=tools::read(yalm_,"max_distance",0.1);
    max_angle_diff=tools::read<double>(yalm_,"max_angle_diff",0.2618);
    auto R_diag=tools::read<std::vector<double>>(yalm_,"R_diag",{1.97e-4,1.97e-4,1.57e-5,2.51e-4});R=Eigen::Matrix<double,4,4>::Zero();R.diagonal()=d_v_to_V(R_diag);X=d_v_to_V(tools::read<std::vector<double>>(yalm_,"X",{0,0,0,0,2.513,0.275,0.102}));
    auto v_P0_diag=tools::read<std::vector<double>>(yalm_,"P0",{0.0025,0.0025,0.01,0.04,1.5,2.5e-5,2.5e-5});P0_diag=d_v_to_V(v_P0_diag);X0=X;N=tools::read(yalm,"N",3);
    }
  else if(goal=="static_armor"){
    v1=tools::read<double>(yalm_,"v1",1e-2);v2=tools::read<double>(yalm_,"v2",5e-1);color=tools::read<std::string>(yalm,"enemy_color","none");max_distance=tools::read(yalm_,"max_distance",0.2);
    max_angle_diff=tools::read<double>(yalm_,"max_angle_diff",0.2618);
    auto R_diag=tools::read<std::vector<double>>(yalm_,"R_diag",{9e-4,9e-4,9e-4});Eigen::Vector3d r3(9e-4,9e-4,9e-4);for(int k=0;k<3&&k<(int)R_diag.size();++k)r3[k]=R_diag[k];
    R=Eigen::Matrix<double,3,3>::Zero();R.diagonal()=r3;
    X=d_v_to_V(tools::read<std::vector<double>>(yalm_,"X",{0,0,0,0,0,0}));
    auto v_P0_diag=tools::read<std::vector<double>>(yalm_,"P0",{0.1,0.1,0.1,100,100,100});P0_diag=d_v_to_V(v_P0_diag);X0=X;N=1;
  }
}
static std::pair<int,int>catch_first(const std::string&goal,const std::vector<auto_aim::Armor>&input,std::string&color,double r,double max_distance,double max_angle_diff,Eigen::Vector2d&center_,const Eigen::Vector3d&cam){
  std::vector<int>index_1=pick_by_color(input,color);
  if(goal=="sentry_station"){
  if(index_1.empty())return{-1,-1};
  if(index_1.size()>1){
    std::vector<Eigen::Vector<double,3>>xyas;for(auto&i:index_1)xyas.push_back(Eigen::Vector<double,3>(input[i].xyz_in_world[0],input[i].xyz_in_world[1],input[i].ypr_in_world[0]));
    auto index_2=pick_armor_double(r,max_distance,max_angle_diff,center_,xyas);
    if(index_2.first!=-1)return {index_1[index_2.first],index_1[index_2.second]};
  }//由于yolo检测到2个装甲板的情况极少,改方案抛弃
  int best=index_1.front();double best_score=-2.0;
  for(auto i:index_1){
    Eigen::Vector2d d(input[i].xyz_in_world[0]-cam[0],input[i].xyz_in_world[1]-cam[1]);
    if(d.norm()<1e-6)continue;
    d.normalize();double yaw=input[i].ypr_in_world[0];
    double score=std::abs(std::cos(yaw)*d[0]+std::sin(yaw)*d[1]);
    if(score>best_score){best_score=score;best=i;}
  }
  double yaw=input[best].ypr_in_world[0];
  Eigen::Vector2d d(cam[0]-input[best].xyz_in_world[0],cam[1]-input[best].xyz_in_world[1]);
  Eigen::Vector2d n(cos(yaw),sin(yaw));
  if(n.dot(d)<0.0)n=-n;
  center_=input[best].xyz_in_world.head<2>()-r*n;
  return {best,-1};
  }
  else if(goal=="static_armor"){
    if(index_1.empty())return {-1,-1};
    else{
      int best=index_1.front();double best_score=-2.0;
      for(auto&i:index_1){
      Eigen::Vector2d d(input[i].xyz_in_world[0]-cam[0],input[i].xyz_in_world[1]-cam[1]);
      if(d.norm()<1e-6)continue;
      d.normalize();double yaw=input[i].ypr_in_world[0];
      double score=std::abs(std::cos(yaw)*d[0]+std::sin(yaw)*d[1]);
      if(score>best_score){best_score=score;best=i;}
      }
      center_=Eigen::Vector2d(input[best].xyz_in_world[0],input[best].xyz_in_world[1]);
      return {best,-1};
    }
  }
  return {-1,-1};
}
static Eigen::MatrixXd diag_to_matrix(const Eigen::VectorXd&input){
  Eigen::MatrixXd output(input.size(),input.size());output.setZero();output.diagonal()=input;return output;
}
static void set_x0(const std::string&goal,Eigen::VectorXd&X,const std::pair<int,int>&index_,const std::vector<Armor>&ob,bool&is_right,const Eigen::Vector2d&center_,double max_distance){
  if(goal=="sentry_station"){
    if(index_.second==-1){
      auto&a_0=ob[index_.first];
      X[0]=center_.x();X[1]=center_.y();
      X[3]=std::atan2(a_0.xyz_in_world[1]-center_.y(),a_0.xyz_in_world[0]-center_.x());
      X[2]=a_0.xyz_in_world[2];
      return;
    }
    auto armor_1=ob[index_.first];auto armor_2=ob[index_.second];
    double x1=armor_1.xyz_in_world[0];double x2=armor_2.xyz_in_world[0];
    double y1=armor_1.xyz_in_world[1];double y2=armor_2.xyz_in_world[1];
    double z1=armor_1.xyz_in_world[2];double z2=armor_2.xyz_in_world[2];
    double dh=X[6];
    if(std::abs(std::abs(z1-z2)-dh)>max_distance&&std::abs(std::abs(z1-z2)-2*dh)>max_distance)is_right=false;//高度差误差大于max_distance不可信
    X[0]=center_.x();X[1]=center_.y();
    X[3]=std::atan2(y1-center_.y(),x1-center_.x());
    X[2]=std::min(z1,z2);
  }
  else if(goal=="static_armor"){
    const auto&a_0=ob[index_.first];
    X[0]=a_0.xyz_in_world[0];X[1]=a_0.xyz_in_world[1];X[2]=a_0.xyz_in_world[2];
    X[3]=0.0;X[4]=0.0;
  }
}
static Eigen::MatrixXd setF(const std::string&goal,double dt){
  if(goal=="sentry_station"){
    auto output=Tool::diag_to_matrix(Eigen::Vector<double,7>::Ones());output(3,4)=dt;
    return output;}
    else if(goal=="static_armor"){
      Eigen::Matrix<double,5,5>F_=Tool::diag_to_matrix(Eigen::Vector<double,5>::Ones());
      F_(0,3)=dt;F_(1,4)=dt;
      return F_;
    }
  return Eigen::Matrix<double,7,7>::Identity().eval();
}
static Eigen::MatrixXd set_Q(const std::string&goal,double v1,double v2,double dt){
  if(goal=="sentry_station"){
    double a_4=std::pow(dt,4)/4;double a_3=std::pow(dt,3)/2;double a_2=dt*dt;
    Eigen::Matrix<double,7,7> Q_=Eigen::Matrix<double,7,7>::Zero();Q_.block(0,0,3,3).diagonal()=Eigen::Vector<double,3>::Constant(v1*dt);
    Q_(3,3)=v2*a_4;Q_(3,4)=v2*a_3;Q_(4,3)=v2*a_3;Q_(4,4)=v2*a_2;
    return Q_;
  }
  else if(goal=="static_armor"){
    Eigen::Matrix<double,5,5> Q_=Eigen::Matrix<double,5,5>::Zero();
    Q_.diagonal()<<v1*dt,v1*dt,v1*dt,v2*dt,v2*dt;
    return Q_;
  }
  return Eigen::Matrix<double,7,7>::Zero().eval();
}
static std::vector<int>catch_armor(const std::string&goal,const std::string&color,const std::vector<Armor>&observation,const Eigen::Vector2d&center_,double max_distance,double r_,double z,double dh){
  auto index_1=pick_by_color(observation,color);
  if(goal=="sentry_station"){
    std::vector<int>output{};
    for(auto i:index_1)
    {
      if(std::abs((center_-observation[i].xyz_in_world.head<2>()).norm()-r_)>max_distance)continue;
      double dz_=std::abs(observation[i].xyz_in_world[2]-z);
      if(dz_>max_distance&&std::abs(dz_-dh)>max_distance&&std::abs(dz_-2*dh)>max_distance)continue;
      output.push_back(i);
    }
    return output;
  }
  else if(goal=="static_armor"){
    std::vector<int>output{};
    Eigen::Vector3d pred(center_.x(),center_.y(),z);
    double best=1e30;int best_id=-1;
    for(auto i:index_1){
      double dlen_=(observation[i].xyz_in_world-pred).norm();
      if(dlen_<=best){best=dlen_;best_id=i;}
    }
    if(best_id!=-1&&best<=max_distance)output.push_back(best_id);
    return output;
  }
  return {};
}
static Eigen::VectorXd set_z(const std::string&goal,const auto_aim::Armor&armor_,const Eigen::VectorXd&state_)
{
  if(goal=="sentry_station"){
    double yaw_=armor_.ypr_in_world[0];
    double phi_=std::atan2(armor_.xyz_in_world[1]-state_[1],armor_.xyz_in_world[0]-state_[0]);
    if(std::abs(tools::limit_rad(yaw_-phi_))>1.5708)yaw_=yaw_revolve(yaw_);
    return Eigen::Vector<double,4>(armor_.xyz_in_world[0],armor_.xyz_in_world[1],armor_.xyz_in_world[2],yaw_);}
  else if(goal=="static_armor"){
    return Eigen::Vector<double,3>(armor_.xyz_in_world[0],armor_.xyz_in_world[1],armor_.xyz_in_world[2]);
  }
  return Eigen::VectorXd::Zero(4);
}
static std::pair<int,int> set_id(const std::string&goal,const Eigen::VectorXd&armor_,const Eigen::VectorXd&state){
  if(goal=="sentry_station"){
    double yaw=std::atan2(armor_[1]-state[1],armor_[0]-state[0]);
    int id_=std::lround(tools::limit_rad(yaw-state[3])/2.0944);id_=((id_%3)+3)%3;
    return {id_,std::lround((armor_[2]-state[2])/state[6])};
  }
 else if(goal=="static_armor"){
    return {0,0};
  }
  return {-1,0};
}
static Eigen::MatrixXd set_H(const std::string&goal,const Eigen::VectorXd&state_,int id,int h_id){
  if(goal=="sentry_station"){
    Eigen::Matrix<double,4,7> H_=Eigen::Matrix<double,4,7>::Zero();H_.block(0,0,4,4).diagonal()=Eigen::Vector<double,4>::Ones();
    auto a_=state_[3]+id*2.0944;H_(2,6)=h_id;double sin_=sin(a_);double cos_=cos(a_);
    H_(0,3)=-sin_*state_[5];H_(1,3)=state_[5]*cos_;H_(0,5)=cos_;H_(1,5)=sin_;
    return H_;
  }
  else  if(goal=="static_armor"){
   Eigen::Matrix<double,3,5>H_;H_.setZero();H_.block(0,0,3,3).diagonal()=Eigen::Vector3d::Ones();
   return H_;
  }
  return Eigen::Matrix<double,4,7>::Zero().eval();
}
static std::function<Eigen::VectorXd(const Eigen::VectorXd &)>set_h(const std::string&goal,const std::pair<int,int>&ids){
  if(goal=="sentry_station"){
    return [ids](const Eigen::VectorXd&X){double a_=X[3]+ids.first*2.0944;return Eigen::Vector<double,4>(X[0]+X[5]*cos(a_),X[1]+X[5]*sin(a_),X[2]+ids.second*X[6],a_);};
  }
  else if(goal=="static_armor"){
    return [](const Eigen::VectorXd&X){return Eigen::VectorXd(X.head<3>());};
  }
  return [](const Eigen::VectorXd&X){return X;};
}
static std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> set_sub_z(const std::string&goal){
  if(goal=="sentry_station"){
    return [](const Eigen::VectorXd&a,const Eigen::VectorXd&b){Eigen::Vector<double,4>mid=a-b;mid[3]=tools::limit_rad(mid[3]);return mid;};
  }
  else if(goal=="static_armor"){return [](const Eigen::VectorXd&a,const Eigen::VectorXd&b){return Eigen::VectorXd(a-b);};}
  return [](const Eigen::VectorXd&a,const Eigen::VectorXd&b){return Eigen::VectorXd(a-b);};
}
private:
static std::vector<int>pick_by_color(const std::vector<auto_aim::Armor>&input,std::string color){
 std::vector<int>output{};size_t i=0;
 if(color=="none"){for(int i=0;i<input.size();++i)output.push_back(i);}
 else {for(auto&armor:input){if(color_to_string(armor.color)==color)output.push_back(i);++i;}}
 return output;
}
static Eigen::VectorXd d_v_to_V(const std::vector<double>&input){
  if(input.empty())return{};
  Eigen::VectorXd output=Eigen::Map<const Eigen::VectorXd>(input.data(),input.size());
  return output;
}
static std::string color_to_string(const auto_aim::Color&color_){
   if (color_ == auto_aim::Color::red) return "red";
    if (color_ == auto_aim::Color::blue) return "blue";
    if (color_ == auto_aim::Color::purple) return "purple";
    return "none";
}
static std::pair<int,int>pick_armor_double(double r_,double max_distance,double max_angle_diff,Eigen::Vector2d&center_,std::vector<Eigen::Vector<double,3>>&armors){
  for(int j=0;j<armors.size()-1;++j){
    for(int i=j+1;i<armors.size();++i){
      auto O1=armors[i].head<2>();auto O2=armors[j].head<2>();
      if(std::abs((O1-O2).norm()-1.73205*r_)>max_distance)continue;//距离门限
      double d_angle=std::abs(tools::limit_rad(armors[i][2]-armors[j][2]));
      if(d_angle>1.57){if(std::abs(d_angle-2.094)>max_angle_diff)continue;}
      else {armors[i][2]=yaw_revolve(armors[i][2]);}
      auto yaw_to_n=[](double yaw){return Eigen::Vector2d(cos(yaw),sin(yaw));};auto n1=yaw_to_n(armors[i][2]);auto n2=yaw_to_n(armors[j][2]);
      auto c1=O1+n1*r_;auto c2=O2+n2*r_;
      if((c1-c2).norm()>max_distance){armors[i][2]=yaw_revolve(armors[i][2]);armors[j][2]=yaw_revolve(armors[j][2]);if(((O1+yaw_to_n(armors[i][2])*r_)-(O2+yaw_to_n(armors[j][2])*r_)).norm()>max_distance)continue;}
      center_=((O1+(yaw_to_n(armors[i][2]))*r_)+(O2+(yaw_to_n(armors[j][2]))*r_))*0.5;
      return {i,j};
    }
  }
  return {-1,-1};
};
static double yaw_revolve(double yaw){return tools::limit_rad(yaw+3.14159);}
};
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

class Target
{
public:
  explicit Target(const std::string & config_path);

  void reset();
  bool update(const std::vector<Armor> & observations, TimePoint timestamp, const Eigen::Vector3d & cam);

  Prediction predict(TimePoint query_time) const;
private:
  std::string config_path_;
  tools::ExtendedKalmanFilter ekf_;
  bool initialized_ = false;
  TimePoint timestamp_{};
  // TODO(学生): 按自己的建模方案补充内部数据与方法。
  std::vector<int>vel_seat;
  Eigen::VectorXd last_state_;
  Eigen::VectorXd X0;
  std::string goal;
  double v1=0.0;
  double v2=0.0;
  Eigen::MatrixXd R;
  Eigen::VectorXd P0_diag;
  std::string color="none";
  double max_distance;
  double max_angle_diff;
  int N;
public:
  std::vector<int>armor_indexs_seat;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
