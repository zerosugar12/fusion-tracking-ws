#include "csv_logger.hpp"
#include "utils/icecream.hpp"
#include <iomanip>

namespace fusion_tracking
{

CSVLogger::~CSVLogger()
{
  close();
}

bool CSVLogger::init(bool enabled, const std::string & file_path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = enabled;
  if (!enabled_) {
    return false;
  }

  try {
    file_.open(file_path, std::ios::out | std::ios::trunc);
    if (file_.is_open()) {
      writeHeader();
      IC(file_path);
      return true;
    } else {
      IC(file_path);
      return false;
    }
  } catch (const std::exception & e) {
    std::string err = e.what();
    IC(err);
    return false;
  }
}

void CSVLogger::writeHeader()
{
  file_ << "timestamp_sec,dt,sensor_source,is_predicting,"
        << "x_est,y_est,dist_est,bearing_deg,"
        << "vx_est,vy_est,cov_x,cov_y,"
        << "r_cam,r_lidar,"
        << "raw_cam_x,raw_cam_y,raw_lidar_x,raw_lidar_y,"
        << "crowd_density,adaptive_status\n";
}

void CSVLogger::log(const TrackingTelemetryRecord & r)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!file_.is_open()) {
    return;
  }

  file_ << std::fixed << std::setprecision(4)
        << r.timestamp_sec << "," << r.dt << "," << r.sensor_source << "," << (r.is_predicting ? 1 : 0) << ","
        << r.x_est << "," << r.y_est << "," << r.dist_est << "," << r.bearing_deg << ","
        << r.vx_est << "," << r.vy_est << "," << r.cov_x << "," << r.cov_y << ","
        << r.r_cam << "," << r.r_lidar << ","
        << r.raw_cam_x << "," << r.raw_cam_y << "," << r.raw_lidar_x << "," << r.raw_lidar_y << ","
        << r.crowd_density << ",\"" << r.adaptive_status << "\"\n";
}

void CSVLogger::log(double timestamp_sec, double dt, const std::string & sensor_source,
                    bool is_predicting, double x, double y, double dist_est, double bearing_deg,
                    double vx, double vy, double cov_x, double cov_y,
                    double r_cam, double r_lidar,
                    double raw_cx, double raw_cy, double raw_lx, double raw_ly,
                    int crowd_density, const std::string & tuner_status)
{
  TrackingTelemetryRecord record;
  record.timestamp_sec = timestamp_sec;
  record.dt = dt;
  record.sensor_source = sensor_source;
  record.is_predicting = is_predicting;
  record.x_est = x;
  record.y_est = y;
  record.dist_est = dist_est;
  record.bearing_deg = bearing_deg;
  record.vx_est = vx;
  record.vy_est = vy;
  record.cov_x = cov_x;
  record.cov_y = cov_y;
  record.r_cam = r_cam;
  record.r_lidar = r_lidar;
  record.raw_cam_x = raw_cx;
  record.raw_cam_y = raw_cy;
  record.raw_lidar_x = raw_lx;
  record.raw_lidar_y = raw_ly;
  record.crowd_density = crowd_density;
  record.adaptive_status = tuner_status;

  log(record);
}

void CSVLogger::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (file_.is_open()) {
    file_.flush();
  }
}

void CSVLogger::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (file_.is_open()) {
    file_.flush();
    file_.close();
  }
}

bool CSVLogger::isOpen() const
{
  return file_.is_open();
}

} // namespace fusion_tracking
