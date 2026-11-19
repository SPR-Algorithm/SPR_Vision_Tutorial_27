#include "target.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSlot = 2.0 * kPi / 3.0;  // 120°
constexpr int kStateDim = 8;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Plate: x = [x, y, z, vx, vy, vz, yaw, vyaw]
enum PlateIdx { P_X = 0, P_Y, P_Z, P_VX, P_VY, P_VZ, P_YAW, P_VYAW };
// Outpost: x = [cx, cy, theta, omega, r, z0, z1, z2]
//   theta = yaw (pháp tuyến RA NGOÀI) của slot 0; slot k có góc theta + k*120°, độ cao z_k.
enum OutIdx { O_CX = 0, O_CY, O_TH, O_W, O_R, O_Z0, O_Z1, O_Z2 };

// Gập góc về [-π, π]. Bắt buộc cho phần dư yaw, nếu không lệch 2π sẽ bị hiểu là quay 1 vòng.
inline double wrap_pi(double a) { return std::remainder(a, 2.0 * kPi); }

inline double seconds(TimePoint a, TimePoint b) { return std::chrono::duration<double>(a - b).count(); }
}  // namespace

// ============================================================
// Constructor — đọc config (mọi tham số có giá trị mặc định trong Params)
// ============================================================
Target::Target(const std::string &config_path) : config_path_(config_path)
{
  try
  {
    const YAML::Node config = YAML::LoadFile(config_path);
    const YAML::Node cfg = config["target"] ? config["target"] : config;

    mode_ = (cfg["mode"].as<std::string>("plate") == "outpost") ? TargetMode::Outpost : TargetMode::Plate;
    const std::string om = cfg["output_mode"].as<std::string>("predict");
    output_mode_ = (om == "raw") ? OutputMode::Raw : (om == "filter") ? OutputMode::Filter : OutputMode::Predict;

    // reset_dt mặc định phụ thuộc chế độ: outpost quay đều nên coasting được lâu hơn.
    prm_.reset_dt = (mode_ == TargetMode::Outpost) ? 0.6 : 0.3;

    auto d = [&](const char *k, double &v) { v = cfg[k].as<double>(v); };
    d("fx", prm_.fx);
    d("sigma_pix", prm_.sigma_pix);
    d("plate_width", prm_.plate_width);
    d("depth_scale", prm_.depth_scale);
    d("sigma_floor", prm_.sigma_floor);
    d("R_yaw", prm_.R_yaw);
    d("gate_chi2", prm_.gate_chi2_4);
    d("gate_chi2_3dof", prm_.gate_chi2_3);
    d("reset_dt", prm_.reset_dt);
    d("max_predict_s", prm_.max_predict_s);
    prm_.use_yaw = cfg["use_yaw"].as<bool>(prm_.use_yaw);

    d("sa2_x", prm_.sa2_x);
    d("sa2_y", prm_.sa2_y);
    d("sa2_z", prm_.sa2_z);
    d("sa2_yaw", prm_.sa2_yaw);

    d("r0", prm_.r0);
    d("sigma_alpha", prm_.sigma_alpha);
    d("q_center", prm_.q_center);
    d("q_r", prm_.q_r);
    d("q_z", prm_.q_z);
    d("p_center", prm_.p_center);
    d("p_theta", prm_.p_theta);
    d("p_omega", prm_.p_omega);
    d("p_r", prm_.p_r);
    d("p_z_seen", prm_.p_z_seen);
    d("p_z_unseen", prm_.p_z_unseen);
    prm_.outpost_class_id = cfg["outpost_class_id"].as<int>(prm_.outpost_class_id);
    prm_.yaw_flip = cfg["yaw_flip"].as<bool>(prm_.yaw_flip);

    tools::logger()->info("Target: mode={} output={} use_yaw={} gate={:.2f}",
                          mode_ == TargetMode::Outpost ? "outpost" : "plate", om, prm_.use_yaw,
                          prm_.use_yaw ? prm_.gate_chi2_4 : prm_.gate_chi2_3);
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
  last_obs_xyza_.setZero();
  last_obs_id_ = -1;
  nis_ = 0.0;
  matched_slot_ = -1;
  num_matched_ = 0;
}

// ============================================================
// Quan sát: lọc, dựng vector đo, dựng R
// ============================================================
bool Target::usable(const Armor &a) const
{
  if (!a.xyz_in_world.allFinite()) return false;
  const double yaw = a.ypr_in_world[0];
  if (mode_ == TargetMode::Outpost)
  {
    // Outpost cần yaw để tính tâm lúc khởi tạo.
    if (!std::isfinite(yaw)) return false;
    if (prm_.outpost_class_id >= 0 && a.class_id != prm_.outpost_class_id) return false;
  }
  else if (prm_.use_yaw && !std::isfinite(yaw))
  {
    return false;
  }
  return true;
}

Eigen::Vector4d Target::measure(const Armor &a) const
{
  double yaw = a.ypr_in_world[0];
  if (!std::isfinite(yaw)) yaw = 0.0;
  if (mode_ == TargetMode::Outpost && prm_.yaw_flip) yaw += kPi;
  return Eigen::Vector4d(a.xyz_in_world.x(), a.xyz_in_world.y(), a.xyz_in_world.z(), wrap_pi(yaw));
}

// R phụ thuộc khoảng cách d (tutorial 3.2):
//   ngang  σ_l ≈ d/fx · σ_pix            (vuông góc tia nhìn)
//   sâu    σ_d ≈ d²/(fx·L) · σ_pix       (dọc tia nhìn)
// Xoay về hệ thế giới: R3 = σ_l²·I + (σ_d² − σ_l²)·u·uᵀ, u = tia nhìn (camera ≈ gốc world).
// sigma_floor đặt sàn để không quá tự tin khi ở gần.
Eigen::Matrix4d Target::build_R(const Eigen::Vector3d &p) const
{
  const double d = std::max(p.norm(), 0.3);
  const double sl_px = d / prm_.fx * prm_.sigma_pix;
  const double sd_px = prm_.depth_scale * d * d / (prm_.fx * prm_.plate_width) * prm_.sigma_pix;
  const double fl2 = prm_.sigma_floor * prm_.sigma_floor;
  const double sl2 = sl_px * sl_px + fl2;
  const double sd2 = sd_px * sd_px + fl2;
  const Eigen::Vector3d u = p / d;

  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R.topLeftCorner<3, 3>() = sl2 * Eigen::Matrix3d::Identity() + (sd2 - sl2) * u * u.transpose();
  R(3, 3) = prm_.R_yaw;
  return R;
}

// ============================================================
// Mô hình quan sát h(x, k) và Jacobian H (4x8)
// ============================================================
Eigen::Vector4d Target::h_model(const Eigen::VectorXd &x, int k) const
{
  if (mode_ == TargetMode::Plate)
    return Eigen::Vector4d(x(P_X), x(P_Y), x(P_Z), x(P_YAW));

  const double phi = x(O_TH) + k * kSlot;
  return Eigen::Vector4d(x(O_CX) + x(O_R) * std::cos(phi), x(O_CY) + x(O_R) * std::sin(phi), x(O_Z0 + k),
                         wrap_pi(phi));
}

Eigen::MatrixXd Target::H_model(const Eigen::VectorXd &x, int k) const
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, kStateDim);
  if (mode_ == TargetMode::Plate)
  {
    H(0, P_X) = 1.0;
    H(1, P_Y) = 1.0;
    H(2, P_Z) = 1.0;
    H(3, P_YAW) = 1.0;
    return H;
  }
  const double phi = x(O_TH) + k * kSlot;
  const double c = std::cos(phi), s = std::sin(phi), r = x(O_R);
  H(0, O_CX) = 1.0;
  H(1, O_CY) = 1.0;
  H(0, O_TH) = -r * s;
  H(1, O_TH) = r * c;
  H(3, O_TH) = 1.0;
  H(0, O_R) = c;
  H(1, O_R) = s;
  H(2, O_Z0 + k) = 1.0;
  return H;
}

// ============================================================
// Khởi tạo: chọn tấm gần nhất, đặt x0, P0
// ============================================================
bool Target::initialize(const std::vector<const Armor *> &obs, TimePoint ts)
{
  const Armor *a = *std::min_element(obs.begin(), obs.end(), [](const Armor *p, const Armor *q) {
    return p->xyz_in_world.norm() < q->xyz_in_world.norm();
  });
  const Eigen::Vector4d z = measure(*a);
  const Eigen::Matrix4d R = build_R(a->xyz_in_world);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(kStateDim);
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  int yaw_idx;

  if (mode_ == TargetMode::Plate)
  {
    yaw_idx = P_YAW;
    x0(P_X) = z(0);
    x0(P_Y) = z(1);
    x0(P_Z) = z(2);
    x0(P_YAW) = z(3);
    P0.topLeftCorner<3, 3>() = R.topLeftCorner<3, 3>();  // vị trí: khớp R
    P0(P_VX, P_VX) = 16.0;                               // vận tốc chưa biết: bất định lớn
    P0(P_VY, P_VY) = 16.0;
    P0(P_VZ, P_VZ) = 4.0;
    P0(P_YAW, P_YAW) = prm_.R_yaw;
    P0(P_VYAW, P_VYAW) = 25.0;
  }
  else
  {
    // Tấm đầu tiên thấy được coi là slot 0. Tâm = p − r·(cos a, sin a).
    // Nếu tâm trôi theo thời gian: yaw đang hướng VÀO trong -> đặt yaw_flip: true.
    yaw_idx = O_TH;
    x0(O_TH) = z(3);
    x0(O_R) = prm_.r0;
    x0(O_CX) = z(0) - prm_.r0 * std::cos(z(3));
    x0(O_CY) = z(1) - prm_.r0 * std::sin(z(3));
    x0(O_Z0) = x0(O_Z1) = x0(O_Z2) = z(2);
    x0(O_W) = 0.0;
    P0(O_CX, O_CX) = P0(O_CY, O_CY) = prm_.p_center * prm_.p_center;
    P0(O_TH, O_TH) = prm_.p_theta * prm_.p_theta;
    P0(O_W, O_W) = prm_.p_omega * prm_.p_omega;
    P0(O_R, O_R) = prm_.p_r * prm_.p_r;
    P0(O_Z0, O_Z0) = prm_.p_z_seen * prm_.p_z_seen;
    P0(O_Z1, O_Z1) = P0(O_Z2, O_Z2) = prm_.p_z_unseen * prm_.p_z_unseen;
  }

  // x_add: cộng hiệu chỉnh K·y vào state, có gập góc.
  ekf_ = tools::ExtendedKalmanFilter(x0, P0, [yaw_idx](const Eigen::VectorXd &s, const Eigen::VectorXd &d) {
    Eigen::VectorXd r = s + d;
    r(yaw_idx) = wrap_pi(r(yaw_idx));
    return r;
  });

  initialized_ = true;
  timestamp_ = ts;
  ++generation_;
  last_obs_xyza_ = z;
  last_obs_id_ = (mode_ == TargetMode::Outpost) ? 0 : a->class_id;
  matched_slot_ = (mode_ == TargetMode::Outpost) ? 0 : -1;
  num_matched_ = 1;
  nis_ = 0.0;
  return true;
}

// ============================================================
// Predict: x⁻ = F x, P⁻ = F P Fᵀ + Q  (dt thực lấy từ timestamp ảnh)
// ============================================================
void Target::predict_step(double dt)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDim, kStateDim);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kStateDim, kStateDim);

  // Cặp (vị trí, vận tốc) với nhiễu gia tốc trắng σ_a² đã rời rạc hóa:
  //   [dt⁴/4  dt³/2; dt³/2  dt²] · σ_a²
  auto add_pair = [&](int p, int v, double sa2) {
    F(p, v) = dt;
    Q(p, p) += sa2 * dt * dt * dt * dt / 4.0;
    Q(p, v) += sa2 * dt * dt * dt / 2.0;
    Q(v, p) += sa2 * dt * dt * dt / 2.0;
    Q(v, v) += sa2 * dt * dt;
  };

  if (mode_ == TargetMode::Plate)
  {
    add_pair(P_X, P_VX, prm_.sa2_x);
    add_pair(P_Y, P_VY, prm_.sa2_y);
    add_pair(P_Z, P_VZ, prm_.sa2_z);
    add_pair(P_YAW, P_VYAW, prm_.sa2_yaw);
  }
  else
  {
    add_pair(O_TH, O_W, prm_.sigma_alpha * prm_.sigma_alpha);
    // Hình học gần như hằng: chỉ cho nhiễu rất nhỏ để hội tụ chậm.
    Q(O_CX, O_CX) = Q(O_CY, O_CY) = prm_.q_center * prm_.q_center * dt;
    Q(O_R, O_R) = prm_.q_r * prm_.q_r * dt;
    Q(O_Z0, O_Z0) = Q(O_Z1, O_Z1) = Q(O_Z2, O_Z2) = prm_.q_z * prm_.q_z * dt;
  }

  ekf_.predict(F, Q);
  const int yi = (mode_ == TargetMode::Plate) ? P_YAW : O_TH;
  ekf_.x(yi) = wrap_pi(ekf_.x(yi));
}

// ============================================================
// Mahalanobis d² = yᵀ S⁻¹ y của quan sát z với giả thuyết slot k (không đổi state)
// ============================================================
double Target::mahalanobis(const Eigen::Vector4d &z, const Eigen::Matrix4d &R, int k) const
{
  const int m = meas_dim();
  Eigen::Vector4d y = z - h_model(ekf_.x, k);
  y(3) = wrap_pi(y(3));
  const Eigen::MatrixXd H = H_model(ekf_.x, k).topRows(m);
  const Eigen::MatrixXd S = H * ekf_.P * H.transpose() + R.topLeftCorner(m, m);
  const Eigen::VectorXd ym = y.head(m);
  const double d2 = ym.dot(S.ldlt().solve(ym));  // giải hệ, không nghịch đảo S
  return std::isfinite(d2) ? d2 : kInf;
}

// ============================================================
// correct với slot k. Cổng đã do ta kiểm; truyền ∞ để EKF không gate lần 2.
// ============================================================
bool Target::apply_correct(const Eigen::Vector4d &z, const Eigen::Matrix4d &R, int k)
{
  const int m = meas_dim();
  const Eigen::MatrixXd H = H_model(ekf_.x, k).topRows(m);
  const Eigen::MatrixXd Rm = R.topLeftCorner(m, m);
  const Eigen::VectorXd zm = z.head(m);

  auto h = [this, k, m](const Eigen::VectorXd &x) -> Eigen::VectorXd { return h_model(x, k).head(m); };
  auto z_sub = [m](const Eigen::VectorXd &a, const Eigen::VectorXd &b) -> Eigen::VectorXd {
    Eigen::VectorXd d = a - b;
    if (m == 4) d(3) = wrap_pi(d(3));  // yaw là thành phần 3 của vector đo
    return d;
  };

  const bool ok = ekf_.correct(zm, H, Rm, h, z_sub, kInf, true);
  if (ok) nis_ = ekf_.last_nis;
  return ok;
}

// ============================================================
// Gán tham lam (greedy) rồi correct.
//   1) tính d² cho mọi cặp (quan sát i, slot k), giữ cặp dưới cổng
//   2) sắp xếp tăng dần
//   3) duyệt: nếu cả quan sát lẫn slot còn trống thì chọn, correct, đánh dấu đã dùng
// Plate chỉ có 1 slot -> thực chất là chọn quan sát tốt nhất.
// ============================================================
bool Target::associate_and_correct(const std::vector<const Armor *> &obs)
{
  const int n_slots = (mode_ == TargetMode::Outpost) ? 3 : 1;
  const int n = static_cast<int>(obs.size());
  const double gate = prm_.use_yaw ? prm_.gate_chi2_4 : prm_.gate_chi2_3;

  std::vector<Eigen::Vector4d> z(n);
  std::vector<Eigen::Matrix4d> R(n);
  for (int i = 0; i < n; ++i)
  {
    z[i] = measure(*obs[i]);
    R[i] = build_R(obs[i]->xyz_in_world);
  }

  struct Cand { double d2; int i; int k; };
  std::vector<Cand> cands;
  for (int i = 0; i < n; ++i)
    for (int k = 0; k < n_slots; ++k)
    {
      const double d2 = mahalanobis(z[i], R[i], k);
      if (d2 < gate) cands.push_back({d2, i, k});  // NaN/inf bị loại vì so sánh sai
    }
  std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) { return a.d2 < b.d2; });

  std::vector<bool> det_used(n, false);
  std::array<bool, 3> slot_used{false, false, false};
  int matched = 0;
  for (const auto &c : cands)
  {
    if (det_used[c.i] || slot_used[c.k]) continue;
    // Sau lần correct trước, state đã đổi nên d² cũ không còn chính xác -> tính lại.
    if (matched > 0 && !(mahalanobis(z[c.i], R[c.i], c.k) < gate)) continue;
    if (!apply_correct(z[c.i], R[c.i], c.k)) continue;

    det_used[c.i] = true;
    slot_used[c.k] = true;
    ++matched;
    matched_slot_ = (mode_ == TargetMode::Outpost) ? c.k : -1;
    last_obs_xyza_ = z[c.i];
    last_obs_id_ = (mode_ == TargetMode::Outpost) ? c.k : obs[c.i]->class_id;
  }
  num_matched_ = matched;
  return matched > 0;
}

// ============================================================
// update — điểm vào chính
// ============================================================
bool Target::update(const std::vector<Armor> &observations, TimePoint timestamp)
{
  std::vector<const Armor *> obs;
  for (const auto &a : observations)
    if (usable(a)) obs.push_back(&a);
  if (obs.empty()) return false;

  if (!initialized_) return initialize(obs, timestamp);

  const double dt = seconds(timestamp, timestamp_);
  if (dt <= 0.0) return false;  // timestamp không tăng: bỏ qua, không chia cho 0
  if (dt > prm_.reset_dt)
  {
    tools::logger()->warn("Target: dt={:.3f}s > {:.2f}s, re-init", dt, prm_.reset_dt);
    reset();
    return initialize(obs, timestamp);  // vẫn dùng quan sát này, không bỏ phí frame
  }

  // 1) PREDICT trước: đẩy state và P tới thời điểm của ảnh.
  predict_step(dt);
  timestamp_ = timestamp;
  num_matched_ = 0;

  // 2) GÁN + CORRECT trên prior đã predict. Nếu không khớp: P đã tăng, cổng frame sau rộng hơn.
  const bool ok = associate_and_correct(obs);

  // 3) Kiểm tra sức khỏe filter.
  bool healthy = ekf_.x.allFinite() && ekf_.P.allFinite();
  if (healthy && mode_ == TargetMode::Outpost)
    healthy = ekf_.x(O_R) > 0.5 * prm_.r0 && ekf_.x(O_R) < 1.5 * prm_.r0;  // r phân kỳ => reset
  if (!healthy)
  {
    tools::logger()->warn("Target: filter diverged, reset");
    reset();
    return false;
  }
  return ok;
}

// ============================================================
// predict — không đổi state. Tuân theo output_mode_ để làm thí nghiệm 3 cách.
// ============================================================
Prediction Target::predict(TimePoint query_time) const
{
  Prediction pred;
  if (!initialized_ || !ekf_.x.allFinite()) return pred;

  double dt = seconds(query_time, timestamp_);
  if (dt < -1e-3 || dt > prm_.max_predict_s) return pred;
  dt = std::max(dt, 0.0);

  if (output_mode_ == OutputMode::Raw)
  {
    // Quan sát thô gần nhất, không lọc, không ngoại suy.
    pred.armors.push_back(PredictedArmor{last_obs_xyza_, last_obs_id_});
    pred.valid = true;
    return pred;
  }
  if (output_mode_ == OutputMode::Filter) dt = 0.0;  // state đã lọc tại thời điểm cập nhật

  Eigen::VectorXd xp = ekf_.x;
  if (mode_ == TargetMode::Plate)
  {
    xp(P_X) += xp(P_VX) * dt;
    xp(P_Y) += xp(P_VY) * dt;
    xp(P_Z) += xp(P_VZ) * dt;
    xp(P_YAW) = wrap_pi(xp(P_YAW) + xp(P_VYAW) * dt);
    pred.armors.push_back(PredictedArmor{h_model(xp, 0), last_obs_id_ >= 0 ? last_obs_id_ : 0});
  }
  else
  {
    xp(O_TH) = wrap_pi(xp(O_TH) + xp(O_W) * dt);
    for (int k = 0; k < 3; ++k) pred.armors.push_back(PredictedArmor{h_model(xp, k), k});
  }
  pred.valid = true;
  return pred;
}

TargetTelemetry Target::telemetry() const
{
  TargetTelemetry t;
  if (!initialized_) return t;
  const auto &x = ekf_.x;
  t.valid = true;
  t.nis = nis_;
  t.matched_slot = matched_slot_;
  t.num_matched = num_matched_;
  if (mode_ == TargetMode::Plate)
  {
    t.pos = Eigen::Vector3d(x(P_X), x(P_Y), x(P_Z));
    t.vel = Eigen::Vector3d(x(P_VX), x(P_VY), x(P_VZ));
    t.yaw = x(P_YAW);
    t.vyaw = x(P_VYAW);
  }
  else
  {
    const int k = std::max(matched_slot_, 0);
    t.pos = Eigen::Vector3d(x(O_CX), x(O_CY), x(O_Z0 + k));
    t.yaw = x(O_TH);
    t.vyaw = x(O_W);
    t.radius = x(O_R);
  }
  return t;
}

}  // namespace auto_aim