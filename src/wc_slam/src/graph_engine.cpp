#include "wc_slam/graph_engine.hpp"

#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/SensorData.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace wc_slam {

namespace {
void require(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}

void valid_pose(const rtabmap::Transform &pose) {
  require(!pose.isNull() && pose.isInvertible(), "null or noninvertible pose");
  for (int index = 0; index < pose.size(); ++index)
    require(std::isfinite(pose[index]), "nonfinite pose");
  const auto rotation = pose.toEigen4d().block<3, 3>(0, 0).eval();
  require((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-4 &&
          std::abs(rotation.determinant() - 1.0) < 1e-4, "pose rotation is not rigid");
}
}  // namespace

GraphEngine::GraphEngine(const std::string &new_database, const EngineOptions &options)
    : max_nodes_(options.max_nodes) {
  require(!new_database.empty() && !std::filesystem::exists(new_database),
          "RTAB-Map database must be a new, nonexisting file");
  require(options.max_nodes > 0 && options.stm_size > 0, "invalid graph resource bounds");
  require(options.icp_voxel_size > 0 && options.icp_max_correspondence > 0 &&
          options.icp_correspondence_ratio > 0 && options.icp_correspondence_ratio <= 1 &&
          options.icp_max_translation > 0 && options.icp_max_rotation > 0 &&
          options.proximity_radius > 0 && options.proximity_angle_degrees > 0 &&
          options.proximity_angle_degrees <= 180, "invalid experimental ICP/proximity settings");
  require(rtabmap::Optimizer::isAvailable(static_cast<rtabmap::Optimizer::Type>(options.optimizer_strategy)),
          "requested RTAB-Map optimizer was not compiled into the installed version");

  parameters_ = {
      {"Mem/GenerateIds", "false"}, {"Mem/IncrementalMemory", "true"},
      {"Mem/ReduceGraph", "false"}, {"Mem/RehearsalSimilarity", "1.0"},
      {"Mem/RehearsalIdUpdatedToNewOne", "false"}, {"Mem/NotLinkedNodesKept", "true"},
      {"Mem/BinDataKept", "true"}, {"Mem/STMSize", std::to_string(options.stm_size)},
      {"DbSqlite3/InMemory", "false"}, {"Rtabmap/DetectionRate", "0"},
      {"Rtabmap/CreateIntermediateNodes", "false"}, {"Rtabmap/MemoryThr", "0"},
      {"Rtabmap/TimeThr", "0"}, {"Kp/MaxFeatures", "-1"},
      {"RGBD/Enabled", "true"}, {"RGBD/LinearUpdate", "0"}, {"RGBD/AngularUpdate", "0"},
      {"RGBD/CreateOccupancyGrid", "false"},  // A fused rig origin is not a free-space sensor origin.
      {"RGBD/OptimizeFromGraphEnd", "false"}, {"RGBD/OptimizeMaxError", "3.0"},
      {"RGBD/ProximityBySpace", "true"}, {"RGBD/ProximityByTime", "false"},
      // Pure ICP needs the existing graph/odometry relative pose as its initial
      // estimate. With the installed default false, Memory::computeTransform
      // takes the visual-feature path for a null guess and scan-only nodes are
      // rejected before ICP. Native registration still verifies every link.
      {"RGBD/ProximityOdomGuess", "true"},
      {"RGBD/ProximityPathMaxNeighbors", "0"},
      {"RGBD/LocalRadius", std::to_string(options.proximity_radius)},
      {"RGBD/ProximityAngle", std::to_string(options.proximity_angle_degrees)},
      {"Optimizer/Strategy", std::to_string(options.optimizer_strategy)}, {"Optimizer/Iterations", "20"},
      {"Reg/Strategy", "1"}, {"Icp/Strategy", "0"}, {"Icp/PointToPlane", "true"},
      {"Icp/PointToPlaneK", "20"}, {"Icp/PointToPlaneMinComplexity", "0.02"},
      {"Icp/PointToPlaneLowComplexityStrategy", "0"},
      {"Icp/VoxelSize", std::to_string(options.icp_voxel_size)},
      {"Icp/MaxCorrespondenceDistance", std::to_string(options.icp_max_correspondence)},
      {"Icp/CorrespondenceRatio", std::to_string(options.icp_correspondence_ratio)},
      {"Icp/MaxTranslation", std::to_string(options.icp_max_translation)},
      {"Icp/MaxRotation", std::to_string(options.icp_max_rotation)}};
  const auto &installed_defaults = rtabmap::Parameters::getDefaultParameters();
  for (const auto &parameter : parameters_)
    require(installed_defaults.count(parameter.first) != 0,
            "installed RTAB-Map does not support parameter: " + parameter.first);
  core_.init(parameters_, new_database, false);
  require(!core_.isIDsGenerated(), "RTAB-Map did not honor explicit input IDs");
}

GraphEngine::~GraphEngine() {
  if (!closed_) {
    try { core_.close(true); } catch (...) { /* Destructors must not throw. */ }
  }
}

EngineSnapshot GraphEngine::process(int explicit_id, double timestamp_seconds,
                                   const cv::Mat &xyz_scan, const rtabmap::Transform &odom_pose,
                                   const cv::Mat &odom_covariance) {
  require(!closed_ && !failed_, "graph engine is closed or latched after a failed update");
  require(explicit_id > last_id_, "bundle IDs must be positive and strictly increasing");
  require(static_cast<int>(odometry_.size()) < max_nodes_, "graph max_nodes reached");
  require(std::isfinite(timestamp_seconds) && timestamp_seconds >= 0, "invalid scan timestamp");
  require(xyz_scan.type() == CV_32FC3 && xyz_scan.rows == 1 && xyz_scan.cols >= 20,
          "graph registration requires a 1xN XYZ scan with at least 20 points");
  require(cv::checkRange(xyz_scan), "nonfinite XYZ scan");
  valid_pose(odom_pose);
  require(odom_covariance.rows == 6 && odom_covariance.cols == 6 && odom_covariance.type() == CV_64FC1 &&
          cv::checkRange(odom_covariance), "invalid odometry covariance matrix");
  require(cv::norm(odom_covariance - odom_covariance.t()) < 1e-10,
          "odometry covariance must be symmetric");
  cv::Mat eigenvalues;
  require(cv::eigen(odom_covariance, eigenvalues) && eigenvalues.at<double>(5) > 0,
          "odometry covariance must be positive definite; unknown zero covariance is not accepted");

  rtabmap::SensorData data;
  data.setId(explicit_id);
  data.setStamp(timestamp_seconds);
  data.setLaserScan(rtabmap::LaserScan(xyz_scan, xyz_scan.cols, 0.0f,
                    rtabmap::LaserScan::kXYZ, rtabmap::Transform::getIdentity()));
  try {
    require(core_.process(data, odom_pose, odom_covariance),
            "RTAB-Map process did not add this accepted bundle to the graph");
    require(core_.getLastLocationId() == explicit_id, "RTAB-Map changed the explicit bundle/node ID");
    odometry_.emplace(explicit_id, odom_pose);
    EngineSnapshot snapshot;
    core_.getGraph(snapshot.poses, snapshot.edges, true, true, nullptr, false, false, false, false, false, false);
    require(snapshot.poses.size() == odometry_.size(), "optimized graph lost a raw-frame association");
    for (const auto &entry : snapshot.poses) {
      require(odometry_.count(entry.first) == 1, "optimized graph contains an unassociated node");
      valid_pose(entry.second);
    }
    for (const auto &entry : snapshot.edges) {
      require(snapshot.poses.count(entry.second.from()) && snapshot.poses.count(entry.second.to()),
              "graph constraint references an unknown node");
      require(entry.second.isValid(), "graph contains an invalid constraint");
    }
    const auto &statistics = core_.getStatistics();
    snapshot.loop_closure_id = statistics.loopClosureId();
    snapshot.proximity_detection_id = statistics.proximityDetectionId();
    snapshot.statistics = statistics.data();
    snapshot.map_to_odom = snapshot.poses.at(explicit_id) * odom_pose.inverse();
    valid_pose(snapshot.map_to_odom);
    last_id_ = explicit_id;
    return snapshot;
  } catch (...) {
    failed_ = true;
    throw;
  }
}

void GraphEngine::close() {
  if (!closed_) {
    core_.close(true);
    closed_ = true;
  }
}

std::string GraphEngine::edge_type(rtabmap::Link::Type type) {
  switch (type) {
    case rtabmap::Link::kNeighbor: return "neighbor";
    case rtabmap::Link::kGlobalClosure: return "global_closure";
    case rtabmap::Link::kLocalSpaceClosure: return "local_space_closure";
    case rtabmap::Link::kLocalTimeClosure: return "local_time_closure";
    case rtabmap::Link::kUserClosure: return "user_closure";
    case rtabmap::Link::kVirtualClosure: return "virtual_closure";
    case rtabmap::Link::kNeighborMerged: return "neighbor_merged";
    case rtabmap::Link::kPosePrior: return "pose_prior";
    case rtabmap::Link::kLandmark: return "landmark";
    case rtabmap::Link::kGravity: return "gravity";
    default: return "unknown_type_" + std::to_string(static_cast<int>(type));
  }
}

}  // namespace wc_slam
