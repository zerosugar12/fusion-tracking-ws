#pragma once

#include <cmath>
#include <algorithm>
#include <string>

namespace fusion_tracking
{

/**
 * @enum SensorSource
 * @brief Identifies which sensor modality successfully updated the state filter in current cycle.
 */
enum class SensorSource
{
  NONE,
  CAMERA,
  LIDAR,
  FUSED_BOTH,
  DEAD_RECKONING
};

inline std::string sensorSourceToString(SensorSource s)
{
  switch (s) {
    case SensorSource::FUSED_BOTH:     return "FUSED";
    case SensorSource::CAMERA:         return "CAMERA";
    case SensorSource::LIDAR:          return "LIDAR";
    case SensorSource::DEAD_RECKONING: return "PREDICT";
    default:                           return "NONE";
  }
}

/**
 * @struct AdaptiveNoiseOutput
 * @brief Holds adaptive sensor noise covariance values (R_cam & R_lidar) and telemetry status.
 */
struct AdaptiveNoiseOutput
{
  double r_cam{0.0225};                    ///< Camera measurement noise covariance R_cam (m^2)
  double r_lidar{0.0400};                  ///< LiDAR measurement noise covariance R_lidar (m^2)
  std::string dominant_sensor{"BALANCED"}; ///< Dominant sensor selected based on lowest noise
  std::string method{"OPTICAL_MODEL"};     ///< Adaptive tuning method identifier
};

/**
 * @class AnalyticalCovarianceTuner
 * @brief Adapts sensor measurement noise covariances based on Stereo Optics Physics & Crowd Geometry.
 * 
 * 1. Depth Camera Stereo Optics Model:
 *    sigma_depth proportional to (d^2 / (f * B))
 *    Depth estimation error variance grows quadratically with target distance:
 *      R_cam(d) = (R_cam_base + alpha_optics * d^2) / Q_cam
 *    When target is outside camera Field-of-View (Out-of-FOV):
 *      R_cam = 10.0 (Camera influence is practically nullified in Kalman update)
 * 
 * 2. LiDAR Crowd Association Noise Model:
 *      R_lidar(N) = R_lidar_base + beta_crowd * N_crowd
 *    Clamped to the empirical range [0.02, 0.45] m^2.
 */
class AnalyticalCovarianceTuner
{
public:
  AnalyticalCovarianceTuner(double r_cam_base = 0.0225,
                            double alpha_optics = 0.018,
                            double r_lidar_base = 0.040,
                            double beta_crowd = 0.035)
  : r_cam_base_(r_cam_base),
    alpha_optics_(alpha_optics),
    r_lidar_base_(r_lidar_base),
    beta_crowd_(beta_crowd)
  {}

  /**
   * @brief Computes adaptive noise covariances for camera and LiDAR
   * @param distance Euclidean distance to predicted target (meters)
   * @param crowd_count Number of neighboring bystanders/legs
   * @param camera_visible Whether the target is visually associated by camera
   * @param visual_quality Visual detection confidence score [0.0 - 1.0]
   * @return AdaptiveNoiseOutput containing computed R_cam and R_lidar
   */
  AdaptiveNoiseOutput compute(double distance, int crowd_count, bool camera_visible, double visual_quality = 0.85) const
  {
    AdaptiveNoiseOutput out;
    out.method = "ANALYTICAL_OPTICS";

    // 1. Target Out-of-FOV or completely occluded from camera
    if (!camera_visible || visual_quality <= 0.05) {
      out.r_cam = 10.0; // High covariance -> Kalman Gain approaches 0
      out.r_lidar = std::clamp(r_lidar_base_ + beta_crowd_ * static_cast<double>(crowd_count), 0.02, 0.45);
      out.dominant_sensor = "LIDAR_ONLY_OUT_OF_FOV";
      return out;
    }

    // 2. Stereo Optics Depth Error Model: R_cam(d^2)
    const double q_clamped = std::clamp(visual_quality, 0.08, 1.0);
    const double d_sq = distance * distance;
    out.r_cam = (r_cam_base_ + alpha_optics_ * d_sq) / q_clamped;
    out.r_cam = std::clamp(out.r_cam, 0.015, 5.0);

    // 3. LiDAR Crowd Geometric Association Noise Model
    out.r_lidar = r_lidar_base_ + beta_crowd_ * static_cast<double>(crowd_count);
    out.r_lidar = std::clamp(out.r_lidar, 0.02, 0.45);

    // 4. Determine dominant sensor for telemetry & visualization
    if (out.r_cam < out.r_lidar * 0.70) {
      out.dominant_sensor = "CAMERA_DOMINANT";
    } else if (out.r_lidar < out.r_cam * 0.70) {
      out.dominant_sensor = "LIDAR_DOMINANT";
    } else {
      out.dominant_sensor = "BALANCED_FUSION";
    }

    return out;
  }

  void setParameters(double r_cam_base, double alpha_optics, double r_lidar_base, double beta_crowd)
  {
    r_cam_base_ = r_cam_base;
    alpha_optics_ = alpha_optics;
    r_lidar_base_ = r_lidar_base;
    beta_crowd_ = beta_crowd;
  }

private:
  double r_cam_base_{0.0225};
  double alpha_optics_{0.018};
  double r_lidar_base_{0.040};
  double beta_crowd_{0.035};
};

/**
 * @class PersonKalmanFilter
 * @brief 4D Constant Velocity Extended Kalman Filter for Human Kinematics Tracking.
 * 
 * State Vector:
 *   x = [p_x, p_y, v_x, v_y]^T
 *   p_x : Forward position relative to robot (meters)
 *   p_y : Lateral position relative to robot (meters)
 *   v_x : Linear forward velocity of target (m/s)
 *   v_y : Linear lateral velocity of target (m/s)
 * 
 * Kinematic Model: Constant Velocity (CV) with Continuous White Noise Acceleration (Q).
 * Accepts dynamically adjusted measurement noise covariances (R_cam, R_lidar) from the
 * Analytical Stereo Optics Covariance Tuner.
 */
class PersonKalmanFilter
{
public:
  explicit PersonKalmanFilter(double process_noise = 1.2, double default_meas_noise = 0.15)
  : q_var_(process_noise),
    r_var_(default_meas_noise * default_meas_noise)
  {
    reset();
  }

  void reset()
  {
    is_initialized_ = false;
    x_ = 0.0;
    y_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    last_source_ = SensorSource::NONE;
    consecutive_lidar_updates_ = 0;
    consecutive_predict_steps_ = 0;

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        p_[i][j] = 0.0;
      }
    }
    p_[0][0] = 0.25;
    p_[1][1] = 0.25;
    p_[2][2] = 1.00;
    p_[3][3] = 1.00;
  }

  void init(double init_x, double init_y, SensorSource source = SensorSource::CAMERA)
  {
    x_ = init_x;
    y_ = init_y;
    vx_ = 0.0;
    vy_ = 0.0;
    last_source_ = source;
    consecutive_lidar_updates_ = 0;
    consecutive_predict_steps_ = 0;

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        p_[i][j] = 0.0;
      }
    }
    p_[0][0] = 0.05;
    p_[1][1] = 0.05;
    p_[2][2] = 0.50;
    p_[3][3] = 0.50;
    is_initialized_ = true;
  }

  /**
   * @brief State propagation step using Constant Velocity Kinematic Model
   */
  void predict(double dt)
  {
    if (!is_initialized_) return;
    if (dt <= 0.0) dt = 0.033;
    if (dt > 1.5) dt = 1.5;

    x_ += vx_ * dt;
    y_ += vy_ * dt;

    // Slight velocity damping during occlusion
    vx_ *= 0.95;
    vy_ *= 0.95;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    const double q_pos = 0.25 * dt4 * q_var_;
    const double q_pos_vel = 0.5 * dt3 * q_var_;
    const double q_vel = dt2 * q_var_;

    const double p00 = p_[0][0] + dt * (p_[2][0] + p_[0][2]) + dt2 * p_[2][2] + q_pos;
    const double p01 = p_[0][1] + dt * p_[2][1] + dt * p_[0][3] + dt2 * p_[2][3];
    const double p02 = p_[0][2] + dt * p_[2][2] + q_pos_vel;
    const double p03 = p_[0][3] + dt * p_[2][3];

    const double p10 = p_[1][0] + dt * p_[3][0] + dt * p_[1][2] + dt2 * p_[3][2];
    const double p11 = p_[1][1] + dt * (p_[3][1] + p_[1][3]) + dt2 * p_[3][3] + q_pos;
    const double p12 = p_[1][2] + dt * p_[3][2];
    const double p13 = p_[1][3] + dt * p_[3][3] + q_pos_vel;

    const double p20 = p_[2][0] + dt * p_[2][2] + q_pos_vel;
    const double p21 = p_[2][1] + dt * p_[2][3];
    const double p22 = p_[2][2] + q_vel;
    const double p23 = p_[2][3];

    const double p30 = p_[3][0] + dt * p_[3][2];
    const double p31 = p_[3][1] + dt * p_[3][3] + q_pos_vel;
    const double p32 = p_[3][2];
    const double p33 = p_[3][3] + q_vel;

    p_[0][0] = p00; p_[0][1] = p01; p_[0][2] = p02; p_[0][3] = p03;
    p_[1][0] = p10; p_[1][1] = p11; p_[1][2] = p12; p_[1][3] = p13;
    p_[2][0] = p20; p_[2][1] = p21; p_[2][2] = p22; p_[2][3] = p23;
    p_[3][0] = p30; p_[3][1] = p31; p_[3][2] = p32; p_[3][3] = p33;

    consecutive_predict_steps_++;
  }

  /**
   * @brief Generic 2D position measurement update with dynamic covariance
   */
  void updateMeasurement(double mx, double my, double meas_noise_var)
  {
    if (!is_initialized_) {
      init(mx, my);
      return;
    }

    const double yx = mx - x_;
    const double yy = my - y_;

    const double s00 = p_[0][0] + meas_noise_var;
    const double s01 = p_[0][1];
    const double s10 = p_[1][0];
    const double s11 = p_[1][1] + meas_noise_var;

    double det = s00 * s11 - s01 * s10;
    if (std::abs(det) < 1e-6) det = 1e-6;

    const double inv_s00 = s11 / det;
    const double inv_s01 = -s01 / det;
    const double inv_s10 = -s10 / det;
    const double inv_s11 = s00 / det;

    double k[4][2];
    for (int i = 0; i < 4; ++i) {
      k[i][0] = p_[i][0] * inv_s00 + p_[i][1] * inv_s10;
      k[i][1] = p_[i][0] * inv_s01 + p_[i][1] * inv_s11;
    }

    x_ += k[0][0] * yx + k[0][1] * yy;
    y_ += k[1][0] * yx + k[1][1] * yy;
    vx_ += k[2][0] * yx + k[2][1] * yy;
    vy_ += k[3][0] * yx + k[3][1] * yy;

    double p_new[4][4];
    for (int j = 0; j < 4; ++j) {
      p_new[0][j] = (1.0 - k[0][0]) * p_[0][j] - k[0][1] * p_[1][j];
      p_new[1][j] = -k[1][0] * p_[0][j] + (1.0 - k[1][1]) * p_[1][j];
      p_new[2][j] = p_[2][j] - k[2][0] * p_[0][j] - k[2][1] * p_[1][j];
      p_new[3][j] = p_[3][j] - k[3][0] * p_[0][j] - k[3][1] * p_[1][j];
    }
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        p_[i][j] = p_new[i][j];
      }
    }

    consecutive_predict_steps_ = 0;
  }

  void updateCamera(double cx, double cy, double r_cam)
  {
    updateMeasurement(cx, cy, r_cam > 0.0 ? r_cam : r_var_);
    last_source_ = SensorSource::CAMERA;
    consecutive_lidar_updates_ = 0;
  }

  void updateLidar(double lx, double ly, double r_lidar)
  {
    updateMeasurement(lx, ly, r_lidar > 0.0 ? r_lidar : 0.0625);
    last_source_ = SensorSource::LIDAR;
    consecutive_lidar_updates_++;
  }

  void updateFused(double cx, double cy, double r_cam, double lx, double ly, double r_lidar)
  {
    updateMeasurement(cx, cy, r_cam > 0.0 ? r_cam : r_var_);
    updateMeasurement(lx, ly, r_lidar > 0.0 ? r_lidar : 0.0625);
    last_source_ = SensorSource::FUSED_BOTH;
    consecutive_lidar_updates_ = 0;
  }

  // State & Covariance Getters
  bool isInitialized() const { return is_initialized_; }
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getVx() const { return vx_; }
  double getVy() const { return vy_; }
  double getCovX() const { return p_[0][0]; }
  double getCovY() const { return p_[1][1]; }
  SensorSource getLastSource() const { return last_source_; }

private:
  bool is_initialized_{false};
  double x_{0.0};
  double y_{0.0};
  double vx_{0.0};
  double vy_{0.0};
  double p_[4][4]{};
  double q_var_{1.2};
  double r_var_{0.0225};
  SensorSource last_source_{SensorSource::NONE};
  int consecutive_lidar_updates_{0};
  int consecutive_predict_steps_{0};
};

} // namespace fusion_tracking
