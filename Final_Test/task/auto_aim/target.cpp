#include "target.hpp"

namespace auto_aim
{
Target::Target(const std::string & config_path) : config_path_(config_path)
{
  initialized_ = false;
  auto yalm=tools::load(config_path);
  goal=tools::read<std::string>(yalm,"goal","sentry_station");
  Tool::parameter_load (goal,yalm,v1,v2,R,P0_diag,color,max_distance,max_angle_diff,last_state_,X0,N);
  vel_seat.assign(N,-1);
  armor_indexs_seat.assign(N,-1);
}

void Target::reset()
{
  initialized_ = false;
  timestamp_ = TimePoint{};
  vel_seat.assign(N,-1);
  ekf_.P=Tool::diag_to_matrix(P0_diag);
  last_state_=X0;  
  armor_indexs_seat.assign(N,-1);
}

bool Target::update(const std::vector<Armor> & observations, TimePoint timestamp, const Eigen::Vector3d & cam)
{
  // 第一次观测直接初始化状态
  if (!initialized_) {
    if (!observations.empty()) {
      Eigen::Vector2d center_;
      auto ob_double=Tool::catch_first(goal,observations,color,last_state_[5],max_distance,max_angle_diff,center_,cam);
      if(ob_double.first==-1)return false;
      bool is_right=true;
      Tool::set_x0(goal,last_state_,ob_double,observations,is_right,center_,max_distance);
      ekf_.x=last_state_;
      ekf_.P=Tool::diag_to_matrix(P0_diag);
      if(!is_right)return false;
      vel_seat.assign(N,-1);
      vel_seat[0]=std::lround((observations[ob_double.first].xyz_in_world[2]-last_state_[2])/last_state_[6]);
      armor_indexs_seat.assign(N,-1);
      armor_indexs_seat[0]=ob_double.first;
      if(ob_double.second!=-1){
        int id2_=std::lround(tools::limit_rad(std::atan2(observations[ob_double.second].xyz_in_world[1]-last_state_[1],observations[ob_double.second].xyz_in_world[0]-last_state_[0])-last_state_[3])/2.0944);id2_=((id2_%N)+N)%N;
        vel_seat[id2_]=std::lround((observations[ob_double.second].xyz_in_world[2]-last_state_[2])/last_state_[6]);
        armor_indexs_seat[id2_]=ob_double.second;
      }

      initialized_ = true;
      timestamp_ = timestamp;
      return true;
    }
    return false;
  }
  double dt = std::chrono::duration<double>(timestamp - timestamp_).count();
  last_state_=ekf_.predict(Tool::setF(goal,dt),Tool::set_Q(goal,v1,v2,dt));
  timestamp_=timestamp;
  if (!observations.empty()) {
    std::vector<int>index=Tool::catch_armor(goal,color,observations,last_state_.head<2>(),max_distance,last_state_[5],last_state_[2],last_state_[6]);
    armor_indexs_seat.assign(N,-1);
    if(index.empty())return false;
    for(auto&i:index){
    auto ids=Tool::set_id(goal,observations[i].xyz_in_world,last_state_);
    vel_seat[ids.first]=ids.second;
    armor_indexs_seat[ids.first]=i;
    last_state_=ekf_.update(Tool::set_z(goal,observations[i],last_state_),Tool::set_H(goal,last_state_,ids.first,ids.second),R,Tool::set_h(goal,ids),Tool::set_sub_z(goal));
    }
    return true;
  }
  return false;
}

Prediction Target::predict(TimePoint query_time) const
{
  Prediction pred;
  // 未初始化直接返回无效结果
  if (!initialized_) {
    pred.valid = false;
    return pred;
  }

  // 计算查询时刻和当前时刻的时间差
  double dt = std::chrono::duration<double>(query_time - timestamp_).count();
  if (dt < 0.0 || dt > 1.0) {
    pred.valid = false;
    return pred;
  }

  // 匀速外推装甲板位置
  double a_now=Tool::pred_yaw(goal,last_state_,dt);
  for(int i=0;i<N;++i)
  {
    if(vel_seat[i]==-1)continue;
    double angle_=tools::limit_rad(a_now+i*(3.1415926*2/N)); 
    Eigen::Vector3d extrapolate_pos =Tool::pred_xyz(goal,last_state_,angle_,vel_seat[i],dt);
    PredictedArmor armor;
    armor.xyza.head<3>() = extrapolate_pos;
    armor.xyza[3]=tools::limit_rad(angle_+3.14159265358979);
    armor.id =i;
    pred.armors.push_back(armor);
    pred.valid = true;
  }
  return pred;
}
}  // namespace auto_aim
