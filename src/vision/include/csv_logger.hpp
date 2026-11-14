#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <memory>

namespace fusion_tracking
{

/**
 * @struct TrackingTelemetryRecord
 * @brief Structured telemetry record containing state estimation, sensor data, and noise parameters for one iteration.
 */
struct TrackingTelemetryRecord
{
  double timestamp_sec{0.0};
  double dt{0.0};
  std::string sensor_source{"NONE"};
  bool is_predicting{false};

  // EKF State Estimation
  double x_est{0.0};
  double y_est{0.0};
  double dist_est{0.0};
  double bearing_deg{0.0};
  double vx_est{0.0};
  double vy_est{0.0};
  double cov_x{0.0};
  double cov_y{0.0};

  // Adaptive Covariance Tuning Parameters
  double r_cam{0.0};
  double r_lidar{0.0};

  // Raw Sensor Measurements
  double raw_cam_x{0.0};
  double raw_cam_y{0.0};
  double raw_lidar_x{0.0};
  double raw_lidar_y{0.0};

  // Crowd / Clutter Density & Status
  int crowd_density{0};
  std::string adaptive_status{"IDLE"};
};

/**
 * @class CSVLogger
 * @brief Dedicated telemetry benchmark logger for research evaluation (RMSE, covariance, tracking stability).
 */
class CSVLogger
{
public:
  CSVLogger() = default;
  ~CSVLogger();

  /**
   * @brief Initializes and opens CSV log file if enabled
   * @param enabled Flag indicating whether logging should be performed
   * @param file_path Destination file path for CSV telemetry output
   */
  bool init(bool enabled, const std::string & file_path);

  /**
   * @brief Logs a single iteration telemetry record
   */
  void log(const TrackingTelemetryRecord & record);

  /**
   * @brief Overload for direct parameter logging
   */
  void log(double timestamp_sec, double dt, const std::string & sensor_source,
           bool is_predicting, double x, double y, double dist_est, double bearing_deg,
           double vx, double vy, double cov_x, double cov_y,
           double r_cam, double r_lidar,
           double raw_cx, double raw_cy, double raw_lx, double raw_ly,
           int crowd_density, const std::string & tuner_status);

  void flush();
  void close();
  bool isOpen() const;

private:
  void writeHeader();

  bool enabled_{false};
  std::ofstream file_;
  std::mutex mutex_;
};

} // namespace fusion_tracking
