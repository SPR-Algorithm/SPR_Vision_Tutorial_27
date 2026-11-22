#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
  using TimePoint = std::chrono::steady_clock::time_point;

  struct PredictedArmor
  {
    // Cùng hệ tọa độ thế giới với observation. Đơn vị: m, rad.
    // xyza = [x, y, z, yaw]; yaw = pháp tuyến tấm giáp, chiều RA NGOÀI.
    Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
    int id = -1;  // plate: class_id; outpost: chỉ số slot 0..2
  };

  struct Prediction
  {
    bool valid = false;
    std::vector<PredictedArmor> armors;  // plate: 1 tấm; outpost: 3 tấm (planner tự chọn)
  };

  // plate   : Task 1, tấm giáp tịnh tiến (EKF tuyến tính 8D).
  // outpost : Task 2, tiền đồn quay quanh tâm cố định (EKF 8D phi tuyến).
  // auto    : (cấu hình) chọn lúc khởi tạo: giáp có name == ArmorName::outpost thì dùng outpost.
  enum class TargetMode { Plate, Outpost };

  // So sánh thí nghiệm: quan sát thô / chỉ lọc / lọc + dự đoán.
  enum class OutputMode { Raw, Filter, Predict };

  struct TargetTelemetry
  {
    bool valid = false;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();  // plate: tâm tấm; outpost: (cx, cy, z tấm vừa khớp)
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    double yaw = 0.0;    // plate: yaw;  outpost: theta
    double vyaw = 0.0;   // plate: vyaw; outpost: omega
    double radius = 0.0; // outpost: r
    double nis = 0.0;    // kỳ vọng ≈ số chiều quan sát
    int matched_slot = -1;
    int num_matched = 0;
  };

  class Target
  {
  public:
    explicit Target(const std::string &config_path);

    void reset();

    // true nếu có ít nhất 1 quan sát qua cổng Mahalanobis và đã correct.
    // Trình tự: lọc hợp lệ -> (init) -> PREDICT tới timestamp -> gán tham lam -> CORRECT.
    bool update(const std::vector<Armor> &observations, TimePoint timestamp);

    Prediction predict(TimePoint query_time) const;

    bool is_initialized() const { return initialized_; }

    // Plate: [x,y,z,vx,vy,vz,yaw,vyaw]; Outpost: [cx,cy,theta,omega,r,z0,z1,z2]. Nên dùng telemetry().
    Eigen::VectorXd get_state() const { return ekf_.x; }
    TargetTelemetry telemetry() const;

    TargetMode mode() const { return mode_; }
    OutputMode output_mode() const { return output_mode_; }
    void set_output_mode(OutputMode m) { output_mode_ = m; }

    // Tăng 1 mỗi lần filter được khởi tạo lại (Tracker dùng để phát hiện).
    int generation() const { return generation_; }

  private:
    struct Params
    {
      // ---- nhiễu quan sát (chung) ----
      double fx = 1400.0;
      double sigma_pix = 1.0;
      double plate_width = 0.135;
      double depth_scale = 1.0;
      double sigma_floor = 0.01;
      double R_yaw = 0.0225;
      bool use_yaw = true;
      // Cổng Mahalanobis. Mặc định 99%; 95% = 9.49 / 7.81.
      // Cổng rộng: ít bị khóa khi mục tiêu đổi chuyển động, nhưng dễ nhận outlier hơn.
      double gate_chi2_4 = 13.28;
      double gate_chi2_3 = 11.34;
      double reset_dt = -1.0;               // <0: tự chọn theo chế độ (plate 0.3 s, outpost 0.6 s)
      double max_predict_s = 1.0;
      int reinit_after_rejects = -1;        // <0: tự chọn (plate 5, outpost 15); 0 = tắt

      // ---- plate ----
      double sa2_x = 2.0, sa2_y = 2.0, sa2_z = 0.5, sa2_yaw = 0.5;  // σ_a², (m/s²)²

      // ---- outpost ----
      double r0 = 0.275;
      double sigma_alpha = 30.0;            // rad/s²
      double q_center = 1e-3, q_r = 1e-3, q_z = 1e-3;  // m/sqrt(s). Tâm gần như đứng yên: giữ NHỎ
      double p_center = 0.10, p_theta = 0.30, p_omega = 3.0, p_r = 0.02;
      double p_z_seen = 0.03, p_z_unseen = 0.15;
      bool outpost_by_name = false;         // true: chỉ nhận giáp có name == ArmorName::outpost
      int outpost_class_id = -1;            // >=0: chỉ nhận class_id này
      bool yaw_flip = false;                // true nếu yaw của Solver hướng VÀO trong
      // Hình học đã biết (ý tưởng từ bản B). fix_geometry=true: r và z_k gần như hằng số.
      bool fix_geometry = false;
      std::vector<double> plate_heights;    // độ cao 3 tấm so với tâm quay (m). ĐO LẠI trên thiết bị
      double center_z_prior = std::numeric_limits<double>::quiet_NaN();  // cao độ tâm trong world (m)
      std::vector<double> plate_angles_deg; // mặc định {0,120,240}
    };

    std::string config_path_;
    Params prm_;
    bool auto_mode_ = false;                // cấu hình mode: auto
    TargetMode mode_ = TargetMode::Plate;   // chế độ đang dùng (auto thì chốt lúc khởi tạo)
    bool mode_cfg_outpost_ = false;         // cấu hình mode: outpost
    OutputMode output_mode_ = OutputMode::Predict;

    std::array<double, 3> angles_{0.0, 2.0943951023931953, 4.1887902047863905};  // góc slot so với theta
    std::array<double, 3> heights_{0.0, 0.0, 0.0};
    bool has_height_prior_ = false;         // đủ plate_heights + center_z_prior để nhận slot theo độ cao

    tools::ExtendedKalmanFilter ekf_;
    bool initialized_ = false;
    TimePoint timestamp_{};
    int generation_ = 0;

    Eigen::Vector4d last_obs_xyza_ = Eigen::Vector4d::Zero();
    int last_obs_id_ = -1;
    double nis_ = 0.0;
    int matched_slot_ = -1;
    int num_matched_ = 0;
    int reject_streak_ = 0;

    int meas_dim() const { return prm_.use_yaw ? 4 : 3; }
    double reset_dt() const { return prm_.reset_dt >= 0.0 ? prm_.reset_dt : (mode_ == TargetMode::Outpost ? 0.6 : 0.3); }
    int reinit_limit() const { return prm_.reinit_after_rejects >= 0 ? prm_.reinit_after_rejects : (mode_ == TargetMode::Outpost ? 15 : 5); }

    bool usable(const Armor &a) const;
    Eigen::Vector4d measure(const Armor &a) const;
    Eigen::Matrix4d build_R(const Eigen::Vector3d &p) const;

    Eigen::Vector4d h_model(const Eigen::VectorXd &x, int k) const;
    Eigen::MatrixXd H_model(const Eigen::VectorXd &x, int k) const;

    bool initialize(const std::vector<const Armor *> &obs, TimePoint ts);
    void predict_step(double dt);
    bool associate_and_correct(const std::vector<const Armor *> &obs);
    double mahalanobis(const Eigen::Vector4d &z, const Eigen::Matrix4d &R, int k) const;
    bool apply_correct(const Eigen::Vector4d &z, const Eigen::Matrix4d &R, int k);
  };
}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP