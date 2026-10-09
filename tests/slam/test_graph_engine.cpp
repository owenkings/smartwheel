// Synthetic scans exercise the actual installed RTAB-Map process/optimizer.
// No link or optimized pose is manually inserted into RTAB-Map.
#include "wc_slam/graph_engine.hpp"
#include "wc_slam/io.hpp"

#include <rtabmap/core/Version.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/utilite/ULogger.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
namespace {
void expect(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}
template<typename Function>
void expect_failure(Function function, const std::string &reason) {
  bool failed = false;
  try { function(); } catch (const std::exception &) { failed = true; }
  expect(failed, reason);
}

cv::Mat room_scan(float sensor_x) {
  const int side = 25;
  cv::Mat scan(1, side * side * 3, CV_32FC3);
  int index = 0;
  for (int row = 0; row < side; ++row) {
    for (int column = 0; column < side; ++column) {
      const float a = -2.0f + row / 6.0f;
      const float b = -1.5f + column / 8.0f;
      scan.at<cv::Vec3f>(0, index++) = cv::Vec3f(4.0f - sensor_x, a, b + 1.0f);
      scan.at<cv::Vec3f>(0, index++) = cv::Vec3f(a - sensor_x, 3.0f, b + 1.0f);
      scan.at<cv::Vec3f>(0, index++) = cv::Vec3f(a - sensor_x, b, -0.5f);
    }
  }
  return scan;
}

// Diagnostic output belongs only to this synthetic executable. Never enable
// debug exports or change a live graph's logging/configuration from this test.
void record_frame(std::ostream &log, int id, float true_x, float odom_x,
                  const cv::Mat &scan, const wc_slam::EngineSnapshot &snapshot) {
  const rtabmap::LaserScan laser(scan, scan.cols, 0.0f,
      rtabmap::LaserScan::kXYZ, rtabmap::Transform::getIdentity());
  double min_range = std::numeric_limits<double>::infinity();
  double max_range = 0.0;
  for (int index = 0; index < scan.cols; ++index) {
    const auto &point = scan.at<cv::Vec3f>(0, index);
    const double range = cv::norm(point);
    min_range = std::min(min_range, range);
    max_range = std::max(max_range, range);
  }
  std::ostringstream line;
  line << std::setprecision(10) << "{\"source_mode\":\"synthetic\",\"id\":" << id
       << ",\"true_x\":" << true_x << ",\"input_odom_x\":" << odom_x
       << ",\"optimized_x\":" << snapshot.poses.at(id).x()
       << ",\"map_to_odom\":" << wc_slam::json_string(snapshot.map_to_odom.prettyPrint())
       << ",\"loop_closure_id\":" << snapshot.loop_closure_id
       << ",\"proximity_detection_id\":" << snapshot.proximity_detection_id
       << ",\"scan\":{\"rows\":" << scan.rows << ",\"cols\":" << scan.cols
       << ",\"cv_type\":" << scan.type() << ",\"finite\":" << (cv::checkRange(scan) ? "true" : "false")
       << ",\"format\":" << wc_slam::json_string(laser.formatName())
       << ",\"size\":" << laser.size() << ",\"max_points\":" << laser.maxPoints()
       << ",\"has_normals_at_input\":" << (laser.hasNormals() ? "true" : "false")
       << ",\"min_range_m\":" << min_range << ",\"max_range_m\":" << max_range
       << "},\"edges\":[";
  bool first = true;
  for (const auto &entry : snapshot.edges) {
    if (!first) line << ',';
    first = false;
    line << "{\"from\":" << entry.second.from() << ",\"to\":" << entry.second.to()
         << ",\"type\":" << wc_slam::json_string(wc_slam::GraphEngine::edge_type(entry.second.type()))
         << ",\"transform\":" << wc_slam::json_string(entry.second.transform().prettyPrint()) << '}';
  }
  line << "],\"statistics\":{";
  first = true;
  for (const auto &entry : snapshot.statistics) {
    if (!first) line << ',';
    first = false;
    line << wc_slam::json_string(entry.first) << ':';
    if (std::isfinite(entry.second)) line << entry.second;
    else line << "null";  // A nonfinite native statistic is unavailable, not zero.
  }
  line << "}}";
  log << line.str() << std::endl;
  std::cout << "WCSLAM_FRAME " << line.str() << std::endl;
}
}  // namespace

int main() {
  char template_path[] = "/tmp/wc_slam_core_XXXXXX";
  char *created = ::mkdtemp(template_path);
  if (!created) return 2;
  const fs::path root(created);
  try {
    expect(wc_slam::text_sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "SHA256 implementation disagrees with known vector");
    wc_slam::atomic_json_file(root / "immutable.json", "{\"ok\":true}\n", true);
    const auto original_hash = wc_slam::file_sha256(root / "immutable.json");
    expect_failure([&] { wc_slam::atomic_json_file(root / "immutable.json", "{}", true); }, "immutable JSON was overwritten");
    expect(wc_slam::file_sha256(root / "immutable.json") == original_hash, "failed commit modified prior snapshot");
    fs::create_symlink(root / "immutable.json", root / "link.json");
    expect_failure([&] { wc_slam::file_sha256(root / "link.json"); }, "symlink hash input was accepted");
    {
      wc_slam::ProcessLock lock(root / "shared.lock");
      const auto child = ::fork();
      expect(child >= 0, "fork failed");
      if (child == 0) {
        try { wc_slam::ProcessLock collision(root / "shared.lock"); ::_exit(9); }
        catch (...) { ::_exit(0); }
      }
      int status = 0;
      ::waitpid(child, &status, 0);
      expect(WIFEXITED(status) && WEXITSTATUS(status) == 0, "cross-process graph lock was not exclusive");
    }
    expect(wc_slam::GraphEngine::edge_type(rtabmap::Link::kNeighbor) == "neighbor", "neighbor edge misclassified");
    expect(wc_slam::GraphEngine::edge_type(rtabmap::Link::kLocalSpaceClosure) == "local_space_closure", "spatial closure edge misclassified");
    expect(wc_slam::GraphEngine::edge_type(rtabmap::Link::kUserClosure) == "user_closure", "manual edge disguised as loop closure");

    const auto database = root / "synthetic_revisit.db";
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(std::getenv("WC_SLAM_TEST_DEBUG") ? ULogger::kDebug : ULogger::kInfo);
    std::ofstream frame_log(root / "frames.jsonl", std::ios::out | std::ios::trunc);
    expect(frame_log.is_open(), "cannot create private test diagnostic log");
    wc_slam::EngineOptions options;
    wc_slam::GraphEngine engine(database.string(), options);
    std::cout << "WCSLAM_DIAGNOSTICS root=" << root << " version=" << RTABMAP_VERSION
              << " level=" << (std::getenv("WC_SLAM_TEST_DEBUG") ? "DEBUG" : "INFO")
              << " covariance_source=explicit_synthetic_test_assumption" << std::endl;
    for (const auto &parameter : engine.parameters())
      std::cout << "WCSLAM_PARAMETER " << parameter.first << '=' << parameter.second << std::endl;
    const auto &defaults = rtabmap::Parameters::getDefaultParameters();
    for (const std::string name : {"RGBD/ProximityOdomGuess", "RGBD/ProximityPathFilteringRadius",
                                  "RGBD/ProximityMaxGraphDepth", "Rtabmap/PublishStats",
                                  "Mem/LaserScanNormalK", "Mem/LaserScanNormalRadius"}) {
      const auto override = engine.parameters().find(name);
      const auto fallback = defaults.find(name);
      std::cout << "WCSLAM_EFFECTIVE_PARAMETER " << name << '='
                << (override != engine.parameters().end() ? override->second :
                    fallback != defaults.end() ? fallback->second : "UNAVAILABLE") << std::endl;
    }
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
    const auto initial_scan = room_scan(0);
    const auto origin = rtabmap::Transform::getIdentity();
    expect_failure([&] { engine.process(0, 1.0, initial_scan, origin, covariance); }, "ID zero was accepted");
    expect_failure([&] { engine.process(1, 1.0, initial_scan, origin, cv::Mat::zeros(6, 6, CV_64FC1)); }, "unknown covariance silently became a model");
    wc_slam::EngineSnapshot snapshot;
    int observed_spatial_closures = 0;
    for (int index = 0; index < 31; ++index) {
      // Move out and then genuinely revisit the same synthetic geometry with a
      // small accumulating odometry error. Scans follow true pose, not drift.
      const float position = index <= 15 ? index * 0.12f : (30 - index) * 0.12f;
      const float drift = index * 0.0015f;
      rtabmap::Transform odometry(position + drift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      const auto scan = room_scan(position);
      snapshot = engine.process(index + 1, 10.0 + index, scan, odometry, covariance);
      record_frame(frame_log, index + 1, position, position + drift, scan, snapshot);
      expect(snapshot.poses.size() == static_cast<size_t>(index + 1), "explicit IDs/complete graph association not retained");
      expect(snapshot.poses.count(index + 1) == 1, "requested bundle ID missing from optimized graph");
      for (const auto &edge : snapshot.edges) {
        if (edge.second.type() == rtabmap::Link::kLocalSpaceClosure &&
            std::abs(edge.second.from() - edge.second.to()) > options.stm_size)
          ++observed_spatial_closures;
        expect(edge.second.type() != rtabmap::Link::kUserClosure, "test unexpectedly created a manual constraint");
      }
    }
    std::cout << "{\"source_mode\":\"synthetic\",\"rtabmap_version\":\"" << RTABMAP_VERSION
              << "\",\"node_count\":" << snapshot.poses.size()
              << ",\"observed_nonadjacent_spatial_constraints_across_revisions\":" << observed_spatial_closures
              << ",\"final_pose_x\":" << snapshot.poses.rbegin()->second.x() << "}" << std::endl;
    expect(observed_spatial_closures > 0, "actual RTAB-Map did not verify a nonadjacent synthetic spatial revisit");
    expect_failure([&] { engine.process(31, 99.0, initial_scan, origin, covariance); }, "duplicate bundle ID accepted");
    engine.close();
    expect(fs::is_regular_file(database) && fs::file_size(database) > 0, "closed RTAB-Map database was not persisted");
    expect_failure([&] { wc_slam::GraphEngine duplicate(database.string(), options); }, "existing database was opened for mutation");
    expect_failure([&] { engine.process(32, 100.0, initial_scan, origin, covariance); }, "closed graph accepted a new frame");
    fs::remove_all(root);  // Only this test's verified mkdtemp directory.
    std::cout << "wc_slam_core: PASS (SYNTHETIC, not real-device or GUI evidence)" << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "wc_slam_core: FAIL: " << error.what() << "; evidence retained at " << root << std::endl;
    return 1;
  }
}
