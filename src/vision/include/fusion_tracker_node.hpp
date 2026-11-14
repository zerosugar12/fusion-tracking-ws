#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "adaptive_ekf.hpp"
#include "leg_detector.hpp"
#include "csv_logger.hpp"

namespace fusion_tracking
{

/**
 * @struct VisionPerson
 * @brief 3D Human detection structure from ZED Camera (Purely Spatial, 100% ID-Free)
 */
struct VisionPerson
{
  double rel_x{0.0};       ///< Forward distance X relative to robot (meters)
  double rel_y{0.0};       ///< Lateral distance Y relative to robot (meters, +Left, -Right)
  double dist{0.0};        ///< Euclidean distance from robot origin (meters)
  double bearing{0.0};     ///< Azimuth bearing angle (radians)
  double confidence{0.85}; ///< Visual detection confidence score [0.0 - 1.0]
};

/**
 * @class FusionTrackerNode
 * @brief Single Unified ROS 2 Node for ZED Depth Camera and 360 LiDAR Asynchronous Fusion
 *        Based on Adaptive Extended Kalman Filter with Quadratic Optical Depth-Error Modeling.
 */
class FusionTrackerNode : public rclcpp::Node
{
public:
  explicit FusionTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~FusionTrackerNode();

private:
  void initParameters();
  void initTopics();

  bool initInternalCamera();
  void captureCameraFrame();

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onControlLoop();

  void associateAndFuse(const std::vector<VisionPerson> & vision_candidates,
                        const std::vector<LegCluster> & leg_clusters,
                        double dt);

  void publishTrackedPose(double x, double y, double vx, double vy, bool valid);
  void publishMarkers(const std::vector<LegCluster> & clusters,
                      double target_x, double target_y,
                      bool target_active, bool is_predicting,
                      const AdaptiveNoiseOutput & noise_out,
                      double target_dist, double target_bearing_deg);
  void publishRobotVelocity(double target_x, double target_y, bool target_active);
  void recordTelemetry(double dt, const std::string & sensor_source,
                       const AdaptiveNoiseOutput & noise_out,
                       double raw_cx, double raw_cy,
                       double raw_lx, double raw_ly,
                       int crowd_count);

  static std::string bearingToDirectionText(double deg);

  // TF2 Transform Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Node Configuration Parameters
  std::string frame_base_{"base_link"};
  std::string scan_topic_{"/scan"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  double control_rate_hz_{30.0};
  double max_detection_range_{3.5};
  double max_lost_sec_{3.0};
  double occlusion_timeout_sec_{2.0};
  double anti_intervening_margin_{0.45};
  double angle_weight_{0.35};
  bool enable_lidar_fusion_{true};
  bool enable_robot_follow_{false};
  double follow_target_distance_{0.90};

  // Kalman Filter & CSV Logger Parameters
  double kf_process_noise_{1.2};
  bool enable_csv_logging_{true};
  std::string csv_log_path_{"fusion_benchmark.csv"};

  // Stereo Optics Depth Error & LiDAR Crowd Model Parameters
  double analytical_r_cam_base_{0.0225};
  double analytical_alpha_optics_{0.018};
  double analytical_r_lidar_base_{0.040};
  double analytical_beta_crowd_{0.035};

  // 360 LiDAR Leg Segmentation Parameters
  double leg_cluster_dist_{0.15};
  double leg_width_min_{0.04};
  double leg_width_max_{0.40};
  double leg_stride_min_{0.10};
  double leg_stride_max_{0.45};
  int leg_min_points_{3};

  // Robot Follower Controller Parameters
  double follow_kp_linear_{0.70};
  double follow_kp_angular_{1.40};
  double follow_max_linear_vel_{0.55};
  double follow_max_angular_vel_{1.00};

  // ZED Camera Hardware Engine
  sl::Camera zed_;
  sl::Mat zed_point_cloud_;
  sl::Bodies bodies_;
  sl::BodyTrackingRuntimeParameters body_runtime_params_;
  bool internal_camera_ready_{false};

  // ROS Subscriptions, Publishers, & Timers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // Thread-Safe Data Buffers
  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::vector<VisionPerson> latest_vision_candidates_;
  rclcpp::Time last_vision_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_loop_time_{0, 0, RCL_ROS_TIME};

  // Core Fusion Engines: 4D EKF, Leg Detector, & Optical Covariance Tuner
  PersonKalmanFilter kf_{1.2, 0.15};
  std::unique_ptr<LegDetector> leg_detector_;
  AnalyticalCovarianceTuner analytical_tuner_;
  bool is_predicting_{false};

  // CSV Telemetry Logger
  CSVLogger logger_;
};

} // namespace fusion_tracking
