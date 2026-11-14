#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "sensor_msgs/msg/laser_scan.hpp"

namespace fusion_tracking
{

/**
 * @struct LegCluster
 * @brief Represents a segmented leg or paired legs detected from LiDAR point cloud.
 */
struct LegCluster
{
  double x{0.0};              ///< Position X relative to robot base_link (meters)
  double y{0.0};              ///< Position Y relative to robot base_link (meters)
  double dist{0.0};           ///< Euclidean distance from robot origin (meters)
  double bearing{0.0};        ///< Azimuth bearing angle (radians)
  double span{0.0};           ///< Cluster width / physical span (meters)
  int count{0};               ///< Number of LiDAR reflection points in cluster
  bool is_paired_legs{false}; ///< True if formed from two paired human legs
};

/**
 * @class LegDetector
 * @brief Segments and filters human leg clusters from 360-degree 2D LiDAR range scans.
 */
class LegDetector
{
public:
  LegDetector(
    double jump_distance = 0.15,
    double min_leg_span = 0.04,
    double max_leg_span = 0.40,
    double min_pair_sep = 0.10,
    double max_pair_sep = 0.45,
    int min_points_per_cluster = 3,
    double max_range = 4.0)
  : jump_dist_(jump_distance),
    min_leg_span_(min_leg_span),
    max_leg_span_(max_leg_span),
    min_pair_sep_(min_pair_sep),
    max_pair_sep_(max_pair_sep),
    min_pts_(min_points_per_cluster),
    max_range_(max_range)
  {}

  /**
   * @brief Extracts single leg and paired leg candidates transformed to base_link frame
   */
  std::vector<LegCluster> extractLegs(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    double laser_tx = 0.0,
    double laser_ty = 0.0,
    double laser_yaw = 0.0) const;

  /**
   * @brief Evaluates crowd density (number of leg clusters) around a reference position
   * @return Count of detected clusters within specified radius (meters)
   */
  int evaluateCrowdDensity(
    const std::vector<LegCluster> & clusters,
    double ref_x, double ref_y,
    double radius = 1.5) const;

private:
  double jump_dist_;
  double min_leg_span_;
  double max_leg_span_;
  double min_pair_sep_;
  double max_pair_sep_;
  int min_pts_;
  double max_range_;
};

} // namespace fusion_tracking
