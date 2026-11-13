#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace auto_aim
{
// ============================================================
// Constructor — load config, tách R/Q/gate
// ============================================================
Target::Target(const std::string &config_path) : config_path_(config_path)
{
  try
  {
    auto config = YAML::LoadFile(config_path);
    auto cfg = config["target"] ? config["target"] : config;

    // R — nhiễu đo PnP (m² cho x,y,z; rad² cho yaw).
    // Đo từ log PnP khi mục tiêu đứng yên (xem analyze_r.py).
    R_x_   = cfg["R_x"].as<double>(0.0004);    // (2cm)²
    R_y_   = cfg["R_y"].as<double>(0.0004);
    R_z_   = cfg["R_z"].as<double>(0.0009);    // (3cm)² — depth kém hơn
    R_yaw_ = cfg["R_yaw"].as<double>(0.0001);  // (0.01rad)² ≈ (0.57°)²

    // Q — cường độ nhiễu trắng gia tốc (m²/s³ cho x,y,z; rad²/s³ cho yaw).
    // Suy từ gia tốc thực tế của mục tiêu.
    sa2_x_   = cfg["sa2_x"].as<double>(2.0);   // xe mặt đất, gia tốc ngang vừa
    sa2_y_   = cfg["sa2_y"].as<double>(2.0);
    sa2_z_   = cfg["sa2_z"].as<double>(0.5);   // ít nhảy dọc
    sa2_yaw_ = cfg["sa2_yaw"].as<double>(3.0); // con quay có thể khởi quay

    // Gate Mahalanobis — chi-square 95%, 4 DOF = 9.49.
    gate_chi2_ = cfg["gate_chi2"].as<double>(9.49);

    tools::logger()->info(
        "Target: R=({:.4f},{:.4f},{:.4f},{:.5f}) "
        "sa2=({:.2f},{:.2f},{:.2f},{:.2f}) gate_chi2={:.2f}",
        R_x_, R_y_, R_z_, R_yaw_,
        sa2_x_, sa2_y_, sa2_z_, sa2_yaw_, gate_chi2_);
  }
  catch (const std::exception &e)
  {
    tools::logger()->warn("Target config load failed: {}, using defaults", e.what());
  }
}

// ============================================================
// Reset — xóa filter, về trạng thái chưa init
// ============================================================
void Target::reset()
{
  ekf_ = tools::ExtendedKalmanFilter{};
  initialized_ = false;
  timestamp_ = TimePoint{};
  last_armor_id_ = -1;
}

// ============================================================
// Helper: wrap góc về (-π, π]
// ============================================================
double Target::wrap_pi(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

// ============================================================
// Helper: trừ vector, wrap riêng yaw (index 3)
// Bắt buộc cho ekf_.correct — nếu không wrap, khi yaw vượt ±π
// bộ lọc tưởng quay gần 1 vòng → K yaw khổng lồ → nhảy loạn.
// ============================================================
Eigen::VectorXd Target::angle_sub(const Eigen::VectorXd &a,
                                  const Eigen::VectorXd &b)
{
  Eigen::VectorXd d = a - b;
  d(3) = wrap_pi(d(3));  // yaw là index 3 trong measurement [x,y,z,yaw]
  return d;
}

// ============================================================
// build_measurement — từ Armor (hệ world) → vector 4D
// ============================================================
Eigen::VectorXd Target::build_measurement(const Armor &armor) const
{
  Eigen::VectorXd z(MEAS_DIM);
  z(0) = armor.xyz_in_world.x();
  z(1) = armor.xyz_in_world.y();
  z(2) = armor.xyz_in_world.z();
  z(3) = armor.ypr_in_world[0];  // yaw theo convention của Solver
  return z;
}

// ============================================================
// state_to_measurement — chiếu state 8D → measurement 4D
// ============================================================
Eigen::VectorXd Target::state_to_measurement(const Eigen::VectorXd &x) const
{
  // x = [x, y, z, vx, vy, vz, yaw, vyaw]
  Eigen::VectorXd z(MEAS_DIM);
  z(0) = x(0);
  z(1) = x(1);
  z(2) = x(2);
  z(3) = x(6);  // yaw
  return z;
}

// ============================================================
// build_H — Jacobian 4×8, chỉ chọn ra các biến đo được
// ============================================================
Eigen::MatrixXd Target::build_H() const
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  H(0, 0) = 1.0;  // x_tấm ← x
  H(1, 1) = 1.0;  // y_tấm ← y
  H(2, 2) = 1.0;  // z_tấm ← z
  H(3, 6) = 1.0;  // yaw_tấm ← yaw
  return H;
}

// ============================================================
// update — trả bool: true nếu update thực sự xảy ra
// ============================================================
bool Target::update(const std::vector<Armor> &observations,
                    TimePoint timestamp)
{
  if (observations.empty()) return false;

  // ---------- 1. Data association (Mahalanobis gate) ----------
  //
  // Bản chất: dùng precision tổng S = H P⁻ H^T + R để chuẩn hóa
  // innovation. Cửa rộng khi P⁻ lớn (chưa chắc), hẹp khi P⁻ nhỏ
  // (đã khóa). Tự động nới rộng ở xa (R(z) lớn) nếu có R(z) động.
  //
  const Armor *chosen = nullptr;
  double best_score = std::numeric_limits<double>::infinity();

  // R dùng cho gate (cùng R dùng trong correct).
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
  R(0, 0) = R_x_;
  R(1, 1) = R_y_;
  R(2, 2) = R_z_;
  R(3, 3) = R_yaw_;

  for (const auto &obs : observations)
  {
    if (!obs.xyz_in_world.allFinite()) continue;

    if (!initialized_)
    {
      // Chưa init: chọn observation đầu tiên hợp lệ.
      chosen = &obs;
      break;
    }

    // Đã init: dùng Mahalanobis.
    Eigen::VectorXd z = build_measurement(obs);
    Eigen::VectorXd y = angle_sub(z, state_to_measurement(ekf_.x));

    Eigen::MatrixXd H = build_H();
    Eigen::MatrixXd S = H * ekf_.P * H.transpose() + R;

    // Giải S * d = y thay vì nghịch đảo S — nhanh + ổn định số.
    Eigen::VectorXd S_inv_y = S.ldlt().solve(y);
    double d_M2 = y.dot(S_inv_y);

    if (d_M2 > gate_chi2_) continue;  // outlier

    if (d_M2 < best_score)
    {
      best_score = d_M2;
      chosen = &obs;
    }
  }

  if (!chosen) return false;  // không có observation hợp lệ

  Eigen::VectorXd z = build_measurement(*chosen);

  // ---------- 2. Khởi tạo (frame 0) ----------
  //
  // Bản chất: x̂₀ = [z, 0] — vị trí từ observation, vận tốc = 0.
  // P₀ tách:
  //   - Vị trí: khớp R (tin observation khung đầu).
  //   - Vận tốc: LỚN — "tôi không biết vận tốc" → K lớn → học nhanh.
  //   - Yaw: khớp R_yaw.
  //   - Vyaw: LỚN — "tôi không biết vận tốc góc".
  //
  if (!initialized_)
  {
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(STATE_DIM);
    x0(0) = z(0);
    x0(1) = z(1);
    x0(2) = z(2);
    x0(6) = z(3);  // yaw
    // x0(3..5) = 0 (vận tốc), x0(7) = 0 (vyaw)

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    // Vị trí: khớp R.
    P0(0, 0) = R_x_;
    P0(1, 1) = R_y_;
    P0(2, 2) = R_z_;
    // Vận tốc: lớn (bất định cao) — để K lớn ở các frame đầu.
    P0(3, 3) = 16.0;  // vx: (4 m/s)²
    P0(4, 4) = 16.0;  // vy
    P0(5, 5) = 4.0;   // vz: (2 m/s)²
    // Yaw: khớp R_yaw.
    P0(6, 6) = R_yaw_;
    // Vyaw: lớn — con quay có thể khởi quay bất ngờ.
    P0(7, 7) = 25.0;  // (5 rad/s)²

    ekf_ = tools::ExtendedKalmanFilter(
        x0, P0,
        [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return a + b; });

    initialized_ = true;
    timestamp_ = timestamp;
    last_armor_id_ = chosen->class_id;
    return true;
  }

  // ---------- 3. Predict (mỗi frame) ----------
  //
  // Bản chất: P⁻ = F P F^T + Q — bất định TĂNG do thời gian trôi.
  // Q tách 4 cặp (x, y, z, yaw), mỗi cặp dùng công thức rời rạc hóa
  // nhiễu trắng gia tốc.
  //
  double dt = std::chrono::duration<double>(timestamp - timestamp_).count();
  if (dt <= 0.0)
  {
    // Timestamp không tăng — bỏ qua frame này (tránh chia cho 0).
    return false;
  }
  if (dt > 0.2)
  {
    // Mất khung quá lâu → reset thay vì predict sai.
    // (Clamp dt sẽ làm predict thiếu xa và Q thiếu lớn — sai bản chất.)
    tools::logger()->warn("Target: dt={:.3f}s > 0.2s, reset", dt);
    reset();
    return false;
  }

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  F(0, 3) = dt;  // x   ← vx
  F(1, 4) = dt;  // y   ← vy
  F(2, 5) = dt;  // z   ← vz
  F(6, 7) = dt;  // yaw ← vyaw

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  struct AxisQ
  {
    int pos;
    int vel;
    double sa2;
  };
  const AxisQ axes[] = {
      {0, 3, sa2_x_},
      {1, 4, sa2_y_},
      {2, 5, sa2_z_},
      {6, 7, sa2_yaw_},
  };
  for (const auto &a : axes)
  {
    Q(a.pos, a.pos) += a.sa2 * dt * dt * dt * dt/ 4.0;
    Q(a.pos, a.vel) += a.sa2 * dt * dt * dt / 2.0;
    Q(a.vel, a.pos) += a.sa2 * dt * dt * dt / 2.0;
    Q(a.vel, a.vel) += a.sa2 * dt * dt;
  }

  ekf_.predict(F, Q);

  // ---------- 4. Update (kết hợp observation) ----------
  //
  // Bản chất: P GIẢM do có thêm observation. K cân bằng precision
  // prediction và precision observation.
  //
  Eigen::MatrixXd H = build_H();
  // R đã dựng ở bước 1 (dùng lại).

  auto h = [this](const Eigen::VectorXd &x) { return state_to_measurement(x); };

  ekf_.correct(z, H, R, h, angle_sub,
               tools::ExtendedKalmanFilter::nis_threshold(MEAS_DIM), true);

  timestamp_ = timestamp;
  last_armor_id_ = chosen->class_id;

  // Sanity check — nếu filter hỏng (NaN/Inf), reset.
  if (!ekf_.x.allFinite() || !ekf_.P.allFinite())
  {
    tools::logger()->warn("Target: filter state non-finite, reset");
    reset();
    return false;
  }

  return true;
}

// ============================================================
// predict — dự đoán tại query_time, không thay đổi state
// ============================================================
Prediction Target::predict(TimePoint query_time) const
{
  Prediction pred;
  if (!initialized_) return pred;

  double dt = std::chrono::duration<double>(query_time - timestamp_).count();
  if (dt < 0.0 || dt > 1.0) return pred;

  Eigen::VectorXd xp = ekf_.x;
  if (!xp.allFinite()) return pred;

  // Predict thuần (không update) — dùng F với dt query.
  // Bản chất: aim ahead = prediction từ posterior hiện tại.
  xp(0) += xp(3) * dt;  // x   ← vx
  xp(1) += xp(4) * dt;  // y   ← vy
  xp(2) += xp(5) * dt;  // z   ← vz
  xp(6) += xp(7) * dt;  // yaw ← vyaw
  xp(6) = wrap_pi(xp(6));  // wrap yaw sau predict

  PredictedArmor pa;
  pa.xyza = Eigen::Vector4d(xp(0), xp(1), xp(2), xp(6));  // yaw thật!
  pa.id = last_armor_id_ >= 0 ? last_armor_id_ : 0;
  pred.armors.push_back(pa);
  pred.valid = true;
  return pred;
}

}  // namespace auto_aim