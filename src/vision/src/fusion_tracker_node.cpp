#include "fusion_tracker_node.hpp"
#include "utils/icecream.hpp"

#include <sstream>
#include <iomanip>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fusion_tracking
{

// =============================================================================
// 1. LIFECYCLE (CONSTRUCTOR & DESTRUCTOR)
// =============================================================================

FusionTrackerNode::FusionTrackerNode(const rclcpp::NodeOptions & options)
: Node("fusion_tracker_node",
       rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  initParameters();
  initTopics();

  // Initialize Research Telemetry CSV Logger
  logger_.init(enable_csv_logging_, csv_log_path_);

  // Initialize 360 LiDAR Leg Detector with YAML Parameters
  leg_detector_ = std::make_unique<LegDetector>(
    leg_cluster_dist_,
    leg_width_min_,
    leg_width_max_,
    leg_stride_min_,
    leg_stride_max_,
    leg_min_points_,
    max_detection_range_
  );

  // Initialize Extended Kalman Filter (State: [x, y, vx, vy]^T)
  kf_ = PersonKalmanFilter(kf_process_noise_, 0.15);
  last_loop_time_ = now();

  // Initialize Direct ZED Hardware SDK Pipeline
  initInternalCamera();
}

FusionTrackerNode::~FusionTrackerNode()
{
  if (internal_camera_ready_) {
    zed_.disableBodyTracking();
    zed_.disablePositionalTracking();
    zed_.close();
  }
  logger_.close();
}

// =============================================================================
// 2. PARAMETERS & ROS 2 TOPICS INITIALIZATION
// =============================================================================

void FusionTrackerNode::initParameters()
{
  // Frame & Topic Configuration
  get_parameter("frame_base", frame_base_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("control_rate_hz", control_rate_hz_);

  // Spatiotemporal Gating & Anti-Intervening
  get_parameter("max_detection_range", max_detection_range_);
  get_parameter("max_lost_sec", max_lost_sec_);
  get_parameter("occlusion_timeout_sec", occlusion_timeout_sec_);
  get_parameter("anti_intervening_margin", anti_intervening_margin_);
  get_parameter("angle_weight", angle_weight_);

  // Fusion & Follower Modes
  get_parameter("enable_lidar_fusion", enable_lidar_fusion_);
  get_parameter("enable_robot_follow", enable_robot_follow_);
  get_parameter("follow_target_distance", follow_target_distance_);

  // Kalman Filter & CSV Logger Parameters
  get_parameter("kf_process_noise", kf_process_noise_);
  get_parameter("enable_csv_logging", enable_csv_logging_);
  get_parameter("csv_log_path", csv_log_path_);

  // Depth Optics Error Model (d^2) & LiDAR Crowd Noise
  get_parameter("analytical_r_cam_base", analytical_r_cam_base_);
  get_parameter("analytical_alpha_optics", analytical_alpha_optics_);
  get_parameter("analytical_r_lidar_base", analytical_r_lidar_base_);
  get_parameter("analytical_beta_crowd", analytical_beta_crowd_);

  // 360 LiDAR Leg Segmentation Parameters
  get_parameter("leg_cluster_dist", leg_cluster_dist_);
  get_parameter("leg_width_min", leg_width_min_);
  get_parameter("leg_width_max", leg_width_max_);
  get_parameter("leg_stride_min", leg_stride_min_);
  get_parameter("leg_stride_max", leg_stride_max_);
  get_parameter("leg_min_points", leg_min_points_);

  // Proportional Velocity Follower Gains
  get_parameter("follow_kp_linear", follow_kp_linear_);
  get_parameter("follow_kp_angular", follow_kp_angular_);
  get_parameter("follow_max_linear_vel", follow_max_linear_vel_);
  get_parameter("follow_max_angular_vel", follow_max_angular_vel_);
}

void FusionTrackerNode::initTopics()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::default_for_sensor_data();

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FusionTrackerNode::onScan, this, std::placeholders::_1),
    sub_opts);

  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/fusion/tracked_target", rclcpp::QoS(10));
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/fusion/markers", rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  loop_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&FusionTrackerNode::onControlLoop, this));
}

// =============================================================================
// 3. HARDWARE INTEGRATION: ZED CAMERA & RPLIDAR
// =============================================================================

bool FusionTrackerNode::initInternalCamera()
{
  sl::InitParameters init;
  init.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  init.coordinate_units = sl::UNIT::MILLIMETER;
  init.camera_resolution = sl::RESOLUTION::VGA;
  init.camera_fps = static_cast<int>(control_rate_hz_);
  init.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  init.depth_minimum_distance = 300.0f; // 300 mm = 30 cm

  sl::ERROR_CODE state = zed_.open(init);
  if (state != sl::ERROR_CODE::SUCCESS) {
    auto err = sl::toString(state);
    IC(err);
    internal_camera_ready_ = false;
    return false;
  }

  zed_.enablePositionalTracking(sl::PositionalTrackingParameters());

  sl::BodyTrackingParameters body_params;
  body_params.detection_model = sl::BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;
  body_params.body_format = sl::BODY_FORMAT::BODY_34;
  body_params.enable_tracking = true;
  body_params.enable_body_fitting = true;

  if (zed_.enableBodyTracking(body_params) != sl::ERROR_CODE::SUCCESS) {
    zed_.close();
    internal_camera_ready_ = false;
    return false;
  }

  body_runtime_params_.detection_confidence_threshold = 25;
  internal_camera_ready_ = true;
  return true;
}

void FusionTrackerNode::captureCameraFrame()
{
  if (!internal_camera_ready_) return;
  if (zed_.grab() != sl::ERROR_CODE::SUCCESS) return;

  if (zed_.retrieveMeasure(zed_point_cloud_, sl::MEASURE::XYZ, sl::MEM::CPU) != sl::ERROR_CODE::SUCCESS) {
    return;
  }
  if (zed_.retrieveBodies(bodies_, body_runtime_params_) != sl::ERROR_CODE::SUCCESS) {
    return;
  }

  std::vector<VisionPerson> cands;
  for (const auto & body : bodies_.body_list) {
    if (body.keypoint_2d.size() < 34) continue;

    double x = 0.0, y = 0.0;
    if (std::isfinite(body.position.x) && std::isfinite(body.position.y)) {
      // Physical mount offset of ZED Camera on mobile robot base (in millimeters)
      x = (body.position.x + 130.0) / 1000.0;
      y = (-body.position.y + 30.0) / 1000.0;
    }

    const double dist = std::hypot(x, y);
    if (dist < 0.15 || dist > max_detection_range_ || !std::isfinite(dist)) continue;

    VisionPerson p;
    p.rel_x = x;
    p.rel_y = -y;
    p.dist = dist;
    p.bearing = std::atan2(p.rel_y, p.rel_x);
    p.confidence = 0.85;
    cands.push_back(p);
  }

  if (!cands.empty()) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_vision_candidates_ = cands;
    last_vision_time_ = now();
  }
}

void FusionTrackerNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_scan_ = msg;
}

// =============================================================================
// 4. MAIN CONTROL LOOP & ASYNCHRONOUS FUSION ASSOCIATION
// =============================================================================

void FusionTrackerNode::onControlLoop()
{
  if (internal_camera_ready_) {
    captureCameraFrame();
  }

  const auto now_time = now();
  double dt = (now_time - last_loop_time_).seconds();
  last_loop_time_ = now_time;
  if (dt <= 0.0 || dt > 1.0) dt = 1.0 / control_rate_hz_;

  sensor_msgs::msg::LaserScan::SharedPtr scan;
  std::vector<VisionPerson> vision_cands;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    scan = latest_scan_;
    if ((now_time - last_vision_time_).seconds() < 0.4) {
      vision_cands = latest_vision_candidates_;
    }
  }

  // Transform LiDAR frame to base_link coordinates
  double laser_tx = 0.0, laser_ty = 0.0, laser_yaw = 0.0;
  if (scan && !scan->ranges.empty()) {
    try {
      auto tf = tf_buffer_->lookupTransform(frame_base_, scan->header.frame_id, tf2::TimePointZero);
      laser_tx = tf.transform.translation.x;
      laser_ty = tf.transform.translation.y;
      const auto & q = tf.transform.rotation;
      const double siny = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      laser_yaw = std::atan2(siny, cosy);
    } catch (const tf2::TransformException &) {
      laser_yaw = 3.1415926535; // RPLiDAR default physical orientation fallback
    }
  }

  // Segment single/paired human leg clusters from 360-degree LiDAR
  const auto leg_clusters = (scan && !scan->ranges.empty())
    ? leg_detector_->extractLegs(scan, laser_tx, laser_ty, laser_yaw)
    : std::vector<LegCluster>{};

  // Execute Spatiotemporal Data Association & Adaptive EKF Fusion
  associateAndFuse(vision_cands, leg_clusters, dt);
}

void FusionTrackerNode::associateAndFuse(
  const std::vector<VisionPerson> & vision_candidates,
  const std::vector<LegCluster> & leg_clusters,
  double dt)
{
  const auto now_time = now();

  // Initial target lock-on step
  if (!kf_.isInitialized()) {
    int best_v_idx = -1;
    double min_v_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < vision_candidates.size(); ++i) {
      if (vision_candidates[i].dist < min_v_dist && vision_candidates[i].dist <= max_detection_range_) {
        min_v_dist = vision_candidates[i].dist;
        best_v_idx = static_cast<int>(i);
      }
    }

    if (best_v_idx >= 0) {
      const auto & target_init = vision_candidates[best_v_idx];
      kf_.init(target_init.rel_x, target_init.rel_y);
      last_seen_time_ = now_time;
      is_predicting_ = false;
      IC(target_init.dist);
      return;
    }

    int best_l_idx = -1;
    double min_l_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < leg_clusters.size(); ++i) {
      if (leg_clusters[i].dist < min_l_dist && leg_clusters[i].dist <= max_detection_range_) {
        min_l_dist = leg_clusters[i].dist;
        best_l_idx = static_cast<int>(i);
      }
    }

    if (best_l_idx >= 0) {
      const auto & leg_init = leg_clusters[best_l_idx];
      kf_.init(leg_init.x, leg_init.y);
      last_seen_time_ = now_time;
      is_predicting_ = false;
      IC(leg_init.dist);
      return;
    }

    publishTrackedPose(0, 0, 0, 0, false);
    return;
  }

  // 1. KINEMATIC PREDICTION STEP (DEAD-RECKONING PROPAGATION)
  kf_.predict(dt);

  const double pred_x = kf_.getX();
  const double pred_y = kf_.getY();
  const double pred_dist = std::hypot(pred_x, pred_y);
  const double pred_bearing = std::atan2(pred_y, pred_x);

  // 2. SPATIOTEMPORAL GATING & ANTI-INTERVENING BYSTANDER DISQUALIFICATION
  int matched_vision_idx = -1;
  double best_vision_score = std::numeric_limits<double>::infinity();
  const double gating_gate_dist = 0.85 + std::hypot(kf_.getVx(), kf_.getVy()) * 0.6;

  for (size_t i = 0; i < vision_candidates.size(); ++i) {
    const auto & cand = vision_candidates[i];
    const double d_euc = std::hypot(cand.rel_x - pred_x, cand.rel_y - pred_y);

    if (d_euc > gating_gate_dist) continue;

    // Disqualify foreign bystanders cutting in front of tracked target
    if (cand.dist < (pred_dist - anti_intervening_margin_)) continue;

    const double d_angle = std::atan2(std::sin(cand.bearing - pred_bearing),
                                      std::cos(cand.bearing - pred_bearing));
    const double cost = d_euc + angle_weight_ * std::abs(d_angle);
    if (cost < best_vision_score) {
      best_vision_score = cost;
      matched_vision_idx = static_cast<int>(i);
    }
  }

  int matched_lidar_idx = -1;
  double best_lidar_score = std::numeric_limits<double>::infinity();

  for (size_t j = 0; j < leg_clusters.size(); ++j) {
    const auto & cluster = leg_clusters[j];
    const double d_euc = std::hypot(cluster.x - pred_x, cluster.y - pred_y);

    if (d_euc > gating_gate_dist) continue;
    if (cluster.dist < (pred_dist - anti_intervening_margin_)) continue;

    const double d_angle = std::atan2(std::sin(cluster.bearing - pred_bearing),
                                      std::cos(cluster.bearing - pred_bearing));
    const double cost = d_euc + angle_weight_ * std::abs(d_angle);
    if (cost < best_lidar_score) {
      best_lidar_score = cost;
      matched_lidar_idx = static_cast<int>(j);
    }
  }

  // Evaluate crowd density around the predicted target position
  const int nearby_crowd_count = leg_detector_->evaluateCrowdDensity(leg_clusters, pred_x, pred_y, 1.5);

  // 3. STEREO OPTICS QUADRATIC DEPTH NOISE ADAPTATION & LIDAR CROWD TUNING
  const AdaptiveNoiseOutput noise_out = analytical_tuner_.compute(
    pred_dist,
    nearby_crowd_count,
    (matched_vision_idx >= 0)
  );

  SensorSource current_source = SensorSource::NONE;
  double raw_cx = 0.0, raw_cy = 0.0, raw_lx = 0.0, raw_ly = 0.0;

  // 4. MULTI-MODAL MEASUREMENT UPDATE STEP
  if (matched_vision_idx >= 0 && matched_lidar_idx >= 0) {
    // Simultaneous multi-modal fusion (ZED Camera + LiDAR)
    const auto & v = vision_candidates[matched_vision_idx];
    const auto & l = leg_clusters[matched_lidar_idx];
    kf_.updateFused(v.rel_x, v.rel_y, noise_out.r_cam,
                    l.x, l.y, noise_out.r_lidar);
    current_source = SensorSource::FUSED_BOTH;
    last_seen_time_ = now_time;
    is_predicting_ = false;
    raw_cx = v.rel_x; raw_cy = v.rel_y;
    raw_lx = l.x;     raw_ly = l.y;
  } else if (matched_vision_idx >= 0) {
    // Vision camera only
    const auto & v = vision_candidates[matched_vision_idx];
    kf_.updateCamera(v.rel_x, v.rel_y, noise_out.r_cam);
    current_source = SensorSource::CAMERA;
    last_seen_time_ = now_time;
    is_predicting_ = false;
    raw_cx = v.rel_x; raw_cy = v.rel_y;
  } else if (matched_lidar_idx >= 0 && enable_lidar_fusion_) {
    // 360 LiDAR relay when target is out of camera FOV or occluded
    const auto & l = leg_clusters[matched_lidar_idx];
    kf_.updateLidar(l.x, l.y, noise_out.r_lidar);
    current_source = SensorSource::LIDAR;
    last_seen_time_ = now_time;
    is_predicting_ = false;
    raw_lx = l.x; raw_ly = l.y;
  } else {
    // Dead-reckoning mode (Target occluded or lost)
    current_source = SensorSource::DEAD_RECKONING;
    is_predicting_ = true;
    const double lost_duration = (now_time - last_seen_time_).seconds();

    if (lost_duration > max_lost_sec_) {
      IC(lost_duration);
      kf_.reset();
      publishTrackedPose(0, 0, 0, 0, false);
      return;
    }
  }

  // 5. STATE ESTIMATION OUTPUT & TELEMETRY LOGGING
  const double personPosX = kf_.getX();
  const double personPosY = kf_.getY();
  const double personPosDist = std::hypot(personPosX, personPosY);
  const double personTheta = std::atan2(personPosY, personPosX) * 180.0 / 3.14159265;
  const auto sensorSource = sensorSourceToString(current_source);

  // IceCream direct variable logging
  IC(personPosX, personPosY, personPosDist, personTheta, sensorSource);

  publishTrackedPose(personPosX, personPosY, kf_.getVx(), kf_.getVy(), true);
  publishMarkers(leg_clusters, personPosX, personPosY, true, is_predicting_,
                 noise_out, personPosDist, personTheta);
  publishRobotVelocity(personPosX, personPosY, true);

  // 6. Record Telemetry (Delegated to Modular CSV Logger)
  recordTelemetry(dt, sensorSource, noise_out, raw_cx, raw_cy, raw_lx, raw_ly, nearby_crowd_count);
}

// =============================================================================
// 5. ROS 2 TOPIC PUBLISHING, RVIZ2 HUD, & ROBOT MOTION CONTROL
// =============================================================================

void FusionTrackerNode::publishTrackedPose(
  double x, double y, double vx, double vy, bool valid)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_base_;

  if (valid) {
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0.0;
    const double yaw = std::atan2(vy, vx);
    msg.pose.orientation.z = std::sin(yaw * 0.5);
    msg.pose.orientation.w = std::cos(yaw * 0.5);
  }

  target_pose_pub_->publish(msg);
}

void FusionTrackerNode::publishMarkers(
  const std::vector<LegCluster> & clusters,
  double target_x, double target_y,
  bool target_active, bool is_predicting,
  const AdaptiveNoiseOutput & noise_out,
  double target_dist, double target_bearing_deg)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  // 1. LiDAR Leg Clusters
  for (const auto & c : clusters) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = frame_base_;
    m.ns = "lidar_legs";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = c.x;
    m.pose.position.y = c.y;
    m.pose.position.z = 0.15;
    m.scale.x = 0.18; m.scale.y = 0.18; m.scale.z = 0.30;
    m.color.r = 0.2f; m.color.g = 0.8f; m.color.b = 0.9f; m.color.a = 0.5f;
    markers.markers.push_back(m);
  }

  // 2. Tracked Target Cylinder (Human Representation)
  if (target_active) {
    visualization_msgs::msg::Marker tm;
    tm.header.stamp = now();
    tm.header.frame_id = frame_base_;
    tm.ns = "fused_target";
    tm.id = 100;
    tm.type = visualization_msgs::msg::Marker::CYLINDER;
    tm.action = visualization_msgs::msg::Marker::ADD;
    tm.pose.position.x = target_x;
    tm.pose.position.y = target_y;
    tm.pose.position.z = 0.85;
    tm.scale.x = 0.45; tm.scale.y = 0.45; tm.scale.z = 1.70;

    if (is_predicting) {
      tm.color.r = 1.0f; tm.color.g = 0.2f; tm.color.b = 0.2f; tm.color.a = 0.75f; // Red: Occluded/Predicted
    } else {
      tm.color.r = 0.2f; tm.color.g = 1.0f; tm.color.b = 0.3f; tm.color.a = 0.85f; // Green: Active Fusion
    }
    markers.markers.push_back(tm);

    // 3. Telemetry HUD Text Label
    const std::string dir_text = bearingToDirectionText(target_bearing_deg);
    visualization_msgs::msg::Marker lm;
    lm.header.stamp = now();
    lm.header.frame_id = frame_base_;
    lm.ns = "target_label";
    lm.id = 101;
    lm.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    lm.action = visualization_msgs::msg::Marker::ADD;
    lm.pose.position.x = target_x;
    lm.pose.position.y = target_y;
    lm.pose.position.z = 1.95;
    lm.scale.z = 0.25;
    lm.color.r = 1.0f; lm.color.g = 1.0f; lm.color.b = 1.0f; lm.color.a = 1.0f;

    std::stringstream label_ss;
    label_ss << std::fixed << std::setprecision(2)
             << target_dist << "m [" << dir_text << "] | "
             << (is_predicting ? "PREDICT" : "FUSED") << "\n"
             << "Rc: " << noise_out.r_cam << " | Rl: " << noise_out.r_lidar;
    lm.text = label_ss.str();
    markers.markers.push_back(lm);
  }

  marker_pub_->publish(markers);
}

void FusionTrackerNode::publishRobotVelocity(
  double target_x, double target_y, bool target_active)
{
  if (!enable_robot_follow_) return;

  geometry_msgs::msg::Twist cmd;
  if (target_active && !is_predicting_) {
    const double dist = std::hypot(target_x, target_y);
    const double angle = std::atan2(target_y, target_x);
    const double dist_error = dist - follow_target_distance_;

    if (dist_error > 0.1) {
      cmd.linear.x = std::clamp(follow_kp_linear_ * dist_error, 0.0, follow_max_linear_vel_);
    } else if (dist_error < -0.1) {
      cmd.linear.x = std::clamp(follow_kp_linear_ * dist_error, -0.2, 0.0);
    }
    cmd.angular.z = std::clamp(follow_kp_angular_ * angle, -follow_max_angular_vel_, follow_max_angular_vel_);
  }

  cmd_vel_pub_->publish(cmd);
}

void FusionTrackerNode::recordTelemetry(
  double dt, const std::string & sensor_source,
  const AdaptiveNoiseOutput & noise_out,
  double raw_cx, double raw_cy,
  double raw_lx, double raw_ly,
  int crowd_count)
{
  if (!logger_.isOpen()) {
    return;
  }

  TrackingTelemetryRecord record;
  record.timestamp_sec = now().seconds();
  record.dt = dt;
  record.sensor_source = sensor_source;
  record.is_predicting = is_predicting_;
  record.x_est = kf_.getX();
  record.y_est = kf_.getY();
  record.dist_est = std::hypot(record.x_est, record.y_est);
  record.bearing_deg = std::atan2(record.y_est, record.x_est) * 180.0 / 3.14159265;
  record.vx_est = kf_.getVx();
  record.vy_est = kf_.getVy();
  record.cov_x = kf_.getCovX();
  record.cov_y = kf_.getCovY();
  record.r_cam = noise_out.r_cam;
  record.r_lidar = noise_out.r_lidar;
  record.raw_cam_x = raw_cx;
  record.raw_cam_y = raw_cy;
  record.raw_lidar_x = raw_lx;
  record.raw_lidar_y = raw_ly;
  record.crowd_density = crowd_count;
  record.adaptive_status = noise_out.dominant_sensor;

  logger_.log(record);
}

std::string FusionTrackerNode::bearingToDirectionText(double deg)
{
  if (deg >= -22.5 && deg <= 22.5) return "FRONT";
  if (deg > 22.5 && deg <= 67.5)   return "FRONT-LEFT";
  if (deg > 67.5 && deg <= 112.5)  return "SIDE-LEFT";
  if (deg > 112.5 && deg <= 157.5) return "REAR-LEFT";
  if (deg < -22.5 && deg >= -67.5) return "FRONT-RIGHT";
  if (deg < -67.5 && deg >= -112.5) return "SIDE-RIGHT";
  if (deg < -112.5 && deg >= -157.5) return "REAR-RIGHT";
  return "REAR";
}

} // namespace fusion_tracking

// =============================================================================
// 6. MAIN ENTRY POINT (SINGLE NODE EXECUTABLE)
// =============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<fusion_tracking::FusionTrackerNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
