#include "target.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include "tools/logger.hpp"

namespace auto_aim
{
  Target::Target(const std::string &config_path) : config_path_(config_path)
  {
    try
    {
      auto config = YAML::LoadFile(config_path);
      auto cfg = config["target"] ? config["target"] : config;

      R_pos_pnp_ = cfg["R_pos"].as<double>(R_pos_pnp_);
      // Cường độ nhiễu gia tốc trắng (m^2/s^3) cho Q
      sa2_pos_ = cfg["sa2_pos"].as<double>(5.0);
      // Gate data association
      gate_dist2_ = cfg["gate_dist2"].as<double>(25.0); // (5m)^2

      tools::logger()->info("Target(CV): R_pos={}, sa2_pos={}, gate_dist2={}",
                            R_pos_pnp_, sa2_pos_, gate_dist2_);
    }
    catch (const std::exception &e)
    {
      tools::logger()->warn("Target config load failed: {}, using defaults", e.what());
    }
  }

  void Target::reset()
  {
    ekf_ = tools::ExtendedKalmanFilter{};
    initialized_ = false;
    timestamp_ = TimePoint{};

    last_armor_id_ = -1;
    last_yaw_ = 0.0;
  }
  // ---------- Measurement từ Armor (hệ world) ----------
  Eigen::VectorXd Target::build_measurement(const Armor &armor) const
  {
    Eigen::VectorXd z(MEAS_DIM);
    z(0) = armor.xyz_in_world.x();
    z(1) = armor.xyz_in_world.y();
    z(2) = armor.xyz_in_world.z();
    return z;
  }
  Eigen::VectorXd Target::state_to_measurement(const Eigen::VectorXd &x) const
  {
    return x.head<3>();
  }

  Eigen::MatrixXd Target::build_H() const
  {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    return H;
  }
  void Target::update(const std::vector<Armor> &observations, TimePoint timestamp)
  {
    if (observations.empty())
      return;

    // 1. Data association (Gate)
    const Armor *chosen = nullptr;
    double best_score = std::numeric_limits<double>::infinity();

    for (const auto &obs : observations)
    {
      if (!obs.xyz_in_world.allFinite())
        continue;

      double score = 0.0;
      if (initialized_)
      {
        double dist2 = (obs.xyz_in_world - ekf_.x.head<3>()).squaredNorm();
        if (dist2 > gate_dist2_)
          continue; // Loại outlier
        score = dist2;
      }
      if (score < best_score)
      {
        best_score = score;
        chosen = &obs;
      }
    }
    if (!chosen)
      return;

    Eigen::VectorXd z = build_measurement(*chosen);

    // khoi tao state ban dau x0 tu PnP de lam co so tinh P0
    if (!initialized_)
    {
      Eigen::VectorXd x0 = Eigen::VectorXd::Zero(STATE_DIM);
      x0.head<3>() = z; // measurement dau tien, van toc =0

      Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
      P0.block<3, 3>(0, 0) = R_pos_pnp_ * Eigen::Matrix3d::Identity(); //* ma trận đơn vị 3x3 [1 0 0][0 1 0][0 0 1]
      P0.block<3, 3>(3, 3) = 25.0 * Eigen::Matrix3d::Identity();       // Vận tốc bất định (5m/s)^2

      // initialize filter
      ekf_ = tools::ExtendedKalmanFilter(x0, P0, [](const Eigen::VectorXd &a, const Eigen::VectorXd &b)
                                         { return a + b; });
      // cong 2 state, neu cong 2 goc yaw phai wrap

      initialized_ = true;
      timestamp_ = timestamp;
      last_armor_id_ = chosen->class_id;
      return;
    }
    //----------predict state tiep theo-------------- truyen F, Q vao ham predict
    double dt = std::chrono::duration<double>(timestamp - timestamp_).count();
    dt = std::clamp(dt, 1e-3, 0.2);

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    for (int i : {0, 1, 2})
    {
      int v = i + 3;
      Q(i, i) += sa2_pos_ * dt * dt * dt / 3.0;
      Q(i, v) += sa2_pos_ * dt * dt / 2.0;
      Q(v, i) += sa2_pos_ * dt * dt / 2.0;
      Q(v, v) += sa2_pos_ * dt;
    }

    ekf_.predict(F, Q);

    // 4. Update
    Eigen::MatrixXd H = build_H();
    Eigen::MatrixXd R = R_pos_pnp_ * Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM);

    auto h = [this](const Eigen::VectorXd &x)
    { return state_to_measurement(x); };
    auto z_sub = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b)
    { return a - b; };

    ekf_.correct(z, H, R, h, z_sub,
                 tools::ExtendedKalmanFilter::nis_threshold(MEAS_DIM), true);

    timestamp_ = timestamp;
    last_armor_id_ = chosen->class_id;

    if (!ekf_.x.allFinite() || !ekf_.P.allFinite())
    {
      reset();
    }
  }
  Prediction Target::predict(TimePoint query_time) const
  {
    Prediction pred;
    if (!initialized_)
      return pred;

    double dt = std::chrono::duration<double>(query_time - timestamp_).count();
    if (dt < 0.0 || dt > 1.0)
      return pred;

    Eigen::VectorXd xp = ekf_.x;
    if (!xp.allFinite())
      return pred;

    xp(0) += xp(3) * dt;
    xp(1) += xp(4) * dt;
    xp(2) += xp(5) * dt;

    PredictedArmor pa;
    pa.xyza = Eigen::Vector4d(xp(0), xp(1), xp(2), 0.0); // yaw = 0
    pa.id = last_armor_id_ >= 0 ? last_armor_id_ : 0;
    pred.armors.push_back(pa);
    pred.valid = true;
    return pred;
  }
}
  

   
