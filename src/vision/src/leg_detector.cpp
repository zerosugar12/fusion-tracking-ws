#include "leg_detector.hpp"

namespace fusion_tracking
{

std::vector<LegCluster> LegDetector::extractLegs(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan,
  double laser_tx, double laser_ty, double laser_yaw) const
{
  std::vector<LegCluster> clusters;
  if (!scan || scan->ranges.empty()) return clusters;

  struct Pt { double x; double y; };
  std::vector<Pt> current_pts;

  auto finish_cluster = [&](const std::vector<Pt> & pts) {
    if (static_cast<int>(pts.size()) >= min_pts_) {
      double sum_x = 0.0, sum_y = 0.0;
      for (const auto & p : pts) {
        sum_x += p.x;
        sum_y += p.y;
      }
      const double cx = sum_x / pts.size();
      const double cy = sum_y / pts.size();
      const double span = std::hypot(pts.front().x - pts.back().x,
                                     pts.front().y - pts.back().y);
      if (span >= min_leg_span_ && span <= max_leg_span_) {
        const double dist = std::hypot(cx, cy);
        const double bearing = std::atan2(cy, cx);
        clusters.push_back({
          cx, cy, dist, bearing, span, static_cast<int>(pts.size()), false
        });
      }
    }
  };

  const int num_readings = static_cast<int>(scan->ranges.size());
  for (int i = 0; i < num_readings; ++i) {
    const float r = scan->ranges[i];
    if (!std::isfinite(r) || r < 0.15f || r > max_range_) {
      finish_cluster(current_pts);
      current_pts.clear();
      continue;
    }

    const double angle = scan->angle_min + i * scan->angle_increment;
    const double cos_a = std::cos(angle + laser_yaw);
    const double sin_a = std::sin(angle + laser_yaw);
    const double lx = laser_tx + r * cos_a;
    const double ly = laser_ty + r * sin_a;

    if (!current_pts.empty()) {
      const double d_prev = std::hypot(lx - current_pts.back().x, ly - current_pts.back().y);
      if (d_prev > jump_dist_) {
        finish_cluster(current_pts);
        current_pts.clear();
      }
    }
    current_pts.push_back({lx, ly});
  }
  finish_cluster(current_pts);

  // Two-leg pairing: detect when someone stands with two legs separated by min_pair_sep to max_pair_sep
  std::vector<LegCluster> paired_clusters;
  const size_t n_c = clusters.size();
  for (size_t i = 0; i < n_c; ++i) {
    for (size_t j = i + 1; j < n_c; ++j) {
      const double leg_sep = std::hypot(clusters[i].x - clusters[j].x, clusters[i].y - clusters[j].y);
      if (leg_sep >= min_pair_sep_ && leg_sep <= max_pair_sep_) {
        const double mid_x = 0.5 * (clusters[i].x + clusters[j].x);
        const double mid_y = 0.5 * (clusters[i].y + clusters[j].y);
        const double dist = std::hypot(mid_x, mid_y);
        const double bearing = std::atan2(mid_y, mid_x);
        paired_clusters.push_back({
          mid_x, mid_y, dist, bearing,
          clusters[i].span + clusters[j].span + leg_sep,
          clusters[i].count + clusters[j].count,
          true
        });
      }
    }
  }

  // Prepend paired clusters so they are prioritized during data association
  clusters.insert(clusters.begin(), paired_clusters.begin(), paired_clusters.end());
  return clusters;
}

int LegDetector::evaluateCrowdDensity(
  const std::vector<LegCluster> & clusters,
  double ref_x, double ref_y,
  double radius) const
{
  int count = 0;
  for (const auto & cl : clusters) {
    if (std::hypot(cl.x - ref_x, cl.y - ref_y) <= radius) {
      count++;
    }
  }
  return count;
}

} // namespace fusion_tracking
