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

  struct PredictedArmor
  {
    // Cùng hệ tọa độ thế giới với observation. Đơn vị: m, rad.
    // xyza = [x, y, z, yaw]; yaw = hướng pháp tuyến của tấm giáp, chiều RA NGOÀI.
    Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
    int id = -1;  // plate: class_id của giáp; outpost: chỉ số slot 0..2
  };

  struct Prediction
  {
    bool valid = false;
    std::vector<PredictedArmor> armors;  // plate: 1 tấm; outpost: 3 tấm (planner tự chọn tấm)
  };

  // plate   = Task 1: tấm giáp tịnh tiến (EKF tuyến tính 8D).
  // outpost = Task 2: tiền đồn, 3 tấm cách nhau 120°, quay quanh tâm cố định (EKF 8D phi tuyến).
  enum class TargetMode { Plate, Outpost };

  // Dùng cho thí nghiệm so sánh "quan sát thô / chỉ lọc / lọc + dự đoán" (đề Task 1).
  enum class OutputMode { Raw, Filter, Predict };

  // Dữ liệu gỡ lỗi không phụ thuộc cách sắp xếp state bên trong.
  struct TargetTelemetry
  {
    bool valid = false;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();  // plate: tâm tấm; outpost: (cx, cy, z tấm vừa khớp)
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();  // plate: vận tốc; outpost: 0
    double yaw = 0.0;    // plate: yaw tấm; outpost: theta (yaw slot 0)
    double vyaw = 0.0;   // plate: vyaw;    outpost: omega
    double radius = 0.0; // outpost: r
    double nis = 0.0;    // NIS của lần correct gần nhất (kỳ vọng ≈ số chiều quan sát)
    int matched_slot = -1;
    int num_matched = 0;
  };

  class Target
  {
  public:
    explicit Target(const std::string &config_path);

    void reset();

    // Trả về true nếu có ít nhất 1 quan sát hợp lệ qua cổng Mahalanobis và đã correct.
    // Trình tự bên trong: lọc hợp lệ -> (init) -> PREDICT tới timestamp -> gán -> CORRECT.
    // Khi trả về false mà đã qua bước predict, P đã tăng nên cổng tự nới rộng ở frame sau.
    bool update(const std::vector<Armor> &observations, TimePoint timestamp);

    // Dự đoán tại thời điểm tương lai, KHÔNG đổi state hiện tại.
    Prediction predict(TimePoint query_time) const;

    bool is_initialized() const { return initialized_; }

    // Giữ tương thích code cũ. Plate: [x,y,z,vx,vy,vz,yaw,vyaw];
    // Outpost: [cx,cy,theta,omega,r,z0,z1,z2]. Nên dùng telemetry() để debug.
    Eigen::VectorXd get_state() const { return ekf_.x; }
    TargetTelemetry telemetry() const;

    TargetMode mode() const { return mode_; }
    OutputMode output_mode() const { return output_mode_; }
    void set_output_mode(OutputMode m) { output_mode_ = m; }

    // Tăng 1 mỗi lần filter được khởi tạo lại. Tracker dùng để biết filter vừa bị reset nội bộ.
    int generation() const { return generation_; }

  private:
    struct Params
    {
      // ---- mô hình nhiễu quan sát (chung) ----
      double fx = 1400.0;         // px, lấy từ camera_matrix
      double sigma_pix = 1.0;     // px, sai số góc 4 điểm
      double plate_width = 0.135; // m, giáp nhỏ
      double depth_scale = 1.0;   // hệ số hiệu chỉnh sai số chiều sâu (đo thực nghiệm rồi chỉnh)
      double sigma_floor = 0.01;  // m, sàn nhiễu vị trí (cơ cấu, nhòe ảnh, ngoại tham)
      double R_yaw = 0.0225;      // rad², yaw từ PnP nhiễu hơn vị trí
      bool use_yaw = true;        // false: bỏ yaw khỏi cổng và correct (quan sát 3D)
      double gate_chi2_4 = 9.49;  // chi-bình-phương 95%, 4 bậc tự do
      double gate_chi2_3 = 7.81;  // 3 bậc tự do
      double reset_dt = 0.3;      // s, gián đoạn dài hơn thì khởi tạo lại
      double max_predict_s = 1.0; // s, giới hạn ngoại suy tính từ trạng thái hiện tại

      // ---- plate (Task 1): cường độ nhiễu gia tốc σ_a² (đơn vị (m/s²)², (rad/s²)²) ----
      double sa2_x = 2.0, sa2_y = 2.0, sa2_z = 0.5, sa2_yaw = 0.5;

      // ---- outpost (Task 2) ----
      double r0 = 0.275;           // m, bán kính khởi tạo (Ø550). ĐO LẠI trên thiết bị thật
      double sigma_alpha = 30.0;   // rad/s², nhiễu gia tốc góc (quyết định độ bám khi dừng/đảo chiều)
      double q_center = 1e-3, q_r = 1e-3, q_z = 1e-3;  // m/sqrt(s)
      double p_center = 0.10, p_theta = 0.30, p_omega = 3.0, p_r = 0.02;
      double p_z_seen = 0.03, p_z_unseen = 0.15;
      int outpost_class_id = -1;   // >=0: chỉ nhận giáp có class_id này; -1: nhận tất cả
      bool yaw_flip = false;       // true nếu yaw từ Solver hướng VÀO trong (cộng π)
    };

    std::string config_path_;
    Params prm_;
    TargetMode mode_ = TargetMode::Plate;
    OutputMode output_mode_ = OutputMode::Predict;

    tools::ExtendedKalmanFilter ekf_;
    bool initialized_ = false;
    TimePoint timestamp_{};          // thời điểm mà ekf_.x đang biểu diễn
    int generation_ = 0;

    Eigen::Vector4d last_obs_xyza_ = Eigen::Vector4d::Zero();  // quan sát thô gần nhất được chấp nhận
    int last_obs_id_ = -1;
    double nis_ = 0.0;
    int matched_slot_ = -1;
    int num_matched_ = 0;

    int meas_dim() const { return prm_.use_yaw ? 4 : 3; }
    bool usable(const Armor &a) const;
    Eigen::Vector4d measure(const Armor &a) const;
    Eigen::Matrix4d build_R(const Eigen::Vector3d &p) const;

    // Mô hình quan sát cho slot k (plate chỉ có k = 0).
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
