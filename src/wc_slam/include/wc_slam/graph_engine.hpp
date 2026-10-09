#pragma once

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Transform.h>

#include <map>
#include <string>
#include <vector>

namespace wc_slam {

struct EngineOptions {
  int optimizer_strategy = 2;  // GTSAM is present in inspected RTAB-Map 0.23.7.
  int stm_size = 10;
  int max_nodes = 20000;
  double icp_voxel_size = 0.05;
  double icp_max_correspondence = 0.10;
  double icp_correspondence_ratio = 0.30;
  double icp_max_translation = 0.20;
  double icp_max_rotation = 0.35;
  double proximity_radius = 5.0;
  double proximity_angle_degrees = 45.0;
};

struct EngineSnapshot {
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> edges;
  rtabmap::Transform map_to_odom;
  int loop_closure_id = 0;
  int proximity_detection_id = 0;
  std::map<std::string, float> statistics;
};

// Exactly one caller/thread owns an engine and its new, private database.
// This class never calls an odometry estimator or creates manual graph links.
class GraphEngine {
 public:
  GraphEngine(const std::string &new_database, const EngineOptions &options);
  ~GraphEngine();
  GraphEngine(const GraphEngine &) = delete;
  GraphEngine &operator=(const GraphEngine &) = delete;

  EngineSnapshot process(int explicit_id, double timestamp_seconds,
                         const cv::Mat &xyz_scan, const rtabmap::Transform &odom_pose,
                         const cv::Mat &odom_covariance);
  void close();
  const rtabmap::ParametersMap &parameters() const { return parameters_; }
  bool closed() const { return closed_; }
  static std::string edge_type(rtabmap::Link::Type type);

 private:
  rtabmap::Rtabmap core_;
  rtabmap::ParametersMap parameters_;
  int last_id_ = 0;
  int max_nodes_;
  std::map<int, rtabmap::Transform> odometry_;
  bool closed_ = false;
  bool failed_ = false;
};

}  // namespace wc_slam
