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
    // Cùng hệ tọa độ thế giới với observation, đơn vị m, rad.
    Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
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
    explicit Target(const std::string &config_path);

    void reset();

    // Trả về true nếu update thực sự xảy ra (có observation hợp lệ qua gate).
    // Trả về false nếu không có observation hợp lệ (outlier, rỗng, chưa init
    // nhưng observation không hợp lệ).
    // Tracker dùng kết quả này để phân biệt "mất khung" vs "detect sai".
    bool update(const std::vector<Armor> &observations, TimePoint timestamp);

    // Dự đoán tại thời điểm tương lai, không thay đổi state hiện tại.
    Prediction predict(TimePoint query_time) const;

    // Cho Tracker kiểm tra target đã khởi tạo chưa (tránh dùng target_ rỗng).
    bool is_initialized() const { return initialized_; }

    // Lấy state hiện tại để debug (vận tốc, vị trí)
    Eigen::VectorXd get_state() const { return ekf_.x; }

  private:
    std::string config_path_;
    tools::ExtendedKalmanFilter ekf_;
    bool initialized_ = false;
    TimePoint timestamp_{};

    // State 8D: [x, y, z, vx, vy, vz, yaw, vyaw]
    // Layout: 3 vị trí, 3 vận tốc, yaw, vyaw — giữ tương thích với code cũ
    // (chỉ append yaw, vyaw vào cuối).
    static constexpr int STATE_DIM = 8;
    static constexpr int MEAS_DIM = 4;  // [x, y, z, yaw]

    int last_armor_id_ = -1;

    // R (nhiễu đo) — tách 3 trục vị trí + yaw (vì σz > σxy, và yaw riêng).
    double R_x_, R_y_, R_z_, R_yaw_;

    // Q (nhiễu quá trình) — tách 4 cặp (x, y, z, yaw).
    // Đơn vị: m²/s³ cho vị trí, rad²/s³ cho yaw.
    double sa2_x_, sa2_y_, sa2_z_, sa2_yaw_;

    // Gate Mahalanobis — chi-square 95%, 4 DOF = 9.49.
    double gate_chi2_;

    // Hàm dựng measurement từ Armor (hệ world).
    Eigen::VectorXd build_measurement(const Armor &armor) const;

    // Hàm chiếu state → measurement (dùng cho h và innovation).
    Eigen::VectorXd state_to_measurement(const Eigen::VectorXd &x) const;

    // Jacobian H (tuyến tính hóa h tại x̂).
    Eigen::MatrixXd build_H() const;

    // Wrap góc về (-π, π].
    static double wrap_pi(double a);

    // Trừ vector, wrap riêng thành phần yaw (index 3 của measurement).
    // Dùng làm z_sub trong ekf_.correct — bắt buộc cho yaw.
    static Eigen::VectorXd angle_sub(const Eigen::VectorXd &a,
                                     const Eigen::VectorXd &b);
  };
}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP