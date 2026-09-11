"""Exercise the deployed FAST-LIO synchronizer without ROS or hardware.

Compile the actual sync_packages function with minimal queue/message mocks, so
these cases test its control flow rather than a Python reimplementation.
"""

from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "src/third_party/FAST_LIO_ROS2/src/laserMapping.cpp"

PREAMBLE = r"""
#include <deque>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

struct PointXYZI { float curvature = 0.0f; };
struct Cloud { std::vector<PointXYZI> points; };
struct Imu { struct { double stamp; } header; };
using ImuPtr = std::shared_ptr<const Imu>;
struct MeasureGroup {
    std::shared_ptr<Cloud> lidar;
    double lidar_beg_time = 0.0;
    double lidar_end_time = 0.0;
    std::deque<ImuPtr> imu;
};
std::deque<std::shared_ptr<Cloud>> lidar_buffer;
std::deque<double> time_buffer;
std::deque<ImuPtr> imu_buffer;
bool lidar_pushed = false;
double lidar_end_time = 0.0;
double lidar_mean_scantime = 0.0;
double last_timestamp_imu = -1.0;
int scan_num = 0;
double get_time_sec(double stamp) { return stamp; }
"""

HARNESS = r"""
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
std::shared_ptr<Cloud> add_cloud(double stamp, float duration_ms = 0.0f) {
    auto cloud = std::make_shared<Cloud>();
    cloud->points.resize(2);
    cloud->points.back().curvature = duration_ms;
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(stamp);
    return cloud;
}
void add_imu(double stamp) {
    auto imu = std::make_shared<Imu>();
    imu->header.stamp = stamp;
    imu_buffer.push_back(imu);
    last_timestamp_imu = stamp;
}
void setup_unmatched_start() {
    // Measured first stamps in the historical bag's 115-second slice:
    // the earliest available IMU is 76.3166 ms AFTER the first flash cloud.
    add_cloud(1788528816.5019486);
    add_cloud(1788528816.6019486);
    add_imu(1788528816.5782652);
    add_imu(1788528816.65);
}
void unmatched_cloud_is_dropped() {
    setup_unmatched_start();
    MeasureGroup meas;
    require(!sync_packages(meas), "unmatched cloud must not be processed");
    require(meas.imu.empty(), "unmatched cloud must have no IMU");
    require(lidar_buffer.size() == 1 && time_buffer.size() == 1,
            "unmatched cloud and its timestamp must both be consumed");
    require(!lidar_pushed, "dropping unmatched cloud must reset lidar_pushed");
    require(imu_buffer.size() == 2,
            "future IMUs must be retained for the following cloud");
}
void matched_cloud_after_unmatched_progresses() {
    setup_unmatched_start();
    const auto following_cloud = lidar_buffer.back();
    MeasureGroup meas;
    require(!sync_packages(meas), "first unmatched cloud must be skipped");
    require(sync_packages(meas), "next matched cloud must make progress");
    require(meas.lidar == following_cloud, "must process the following cloud");
    require(meas.imu.size() == 1 &&
            meas.imu.front()->header.stamp == 1788528816.5782652,
            "following cloud must receive its earlier IMU");
    require(lidar_buffer.empty() && time_buffer.empty() && !lidar_pushed,
            "matched cloud must leave consistent empty LiDAR queues");
    require(imu_buffer.size() == 1 && imu_buffer.front()->header.stamp == 1788528816.65,
            "future bracket must remain queued");
}
void exact_timestamp_is_consumed() {
    add_cloud(100.0);
    add_imu(100.0);
    add_imu(100.01);
    MeasureGroup meas;
    require(sync_packages(meas), "IMU exactly at cloud end must be accepted");
    require(meas.imu.size() == 1 && meas.imu.front()->header.stamp == 100.0,
            "exact boundary sample must enter the measurement");
    require(imu_buffer.size() == 1 && imu_buffer.front()->header.stamp == 100.01,
            "sample after exact boundary must remain queued");
    require(lidar_buffer.empty() && time_buffer.empty() && !lidar_pushed,
            "exact boundary must complete the cloud");
}
void wait_preserves_pending_cloud() {
    const auto pending_cloud = add_cloud(100.0, 100.0f);
    add_imu(99.99);
    add_imu(100.05);
    MeasureGroup meas;
    require(!sync_packages(meas), "must wait until IMU reaches cloud end");
    require(lidar_buffer.size() == 1 && time_buffer.size() == 1 && lidar_pushed,
            "waiting must preserve pending cloud and timestamp");
    require(imu_buffer.size() == 2, "waiting must not consume earlier IMUs");
    require(!sync_packages(meas), "repeated poll must keep waiting");
    require(lidar_buffer.front() == pending_cloud && imu_buffer.size() == 2,
            "repeated wait must not mutate queued data");
    add_imu(100.11);
    require(sync_packages(meas), "future bracket must release pending cloud");
    require(meas.lidar == pending_cloud && meas.imu.size() == 2,
            "released cloud must retain all earlier IMU samples");
    require(imu_buffer.size() == 1 && imu_buffer.front()->header.stamp == 100.11,
            "future bracket must be retained after release");
    require(lidar_buffer.empty() && time_buffer.empty() && !lidar_pushed,
            "released cloud must complete and reset pending state");
}
int main(int argc, char **argv) {
    try {
        require(argc == 2, "one scenario required");
        const std::string scenario = argv[1];
        if (scenario == "unmatched_cloud_is_dropped") unmatched_cloud_is_dropped();
        else if (scenario == "matched_cloud_after_unmatched_progresses")
            matched_cloud_after_unmatched_progresses();
        else if (scenario == "exact_timestamp_is_consumed") exact_timestamp_is_consumed();
        else if (scenario == "wait_preserves_pending_cloud") wait_preserves_pending_cloud();
        else throw std::runtime_error("unknown scenario");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
"""


@pytest.fixture(scope="module")
def sync_queue_binary(tmp_path_factory):
    compiler = shutil.which("c++") or shutil.which("g++")
    assert compiler, "A C++ compiler is required to test the actual synchronizer"
    source = SOURCE.read_text(encoding="utf-8")
    functions = re.findall(
        r"^bool sync_packages\(MeasureGroup &meas\)\s*\{.*?^\}",
        source, flags=re.MULTILINE | re.DOTALL,
    )
    assert len(functions) == 1, "Expected exactly one sync_packages definition"
    build_dir = tmp_path_factory.mktemp("fastlio_sync_queue")
    harness = build_dir / "sync_queue.cpp"
    binary = build_dir / "sync_queue"
    harness.write_text(PREAMBLE + functions[0] + HARNESS, encoding="utf-8")
    result = subprocess.run(
        [compiler, "-std=c++17", "-Wall", "-Wextra", str(harness), "-o", str(binary)],
        text=True, capture_output=True, timeout=30, check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return binary


@pytest.mark.parametrize("scenario", [
    "unmatched_cloud_is_dropped",
    "matched_cloud_after_unmatched_progresses",
    "exact_timestamp_is_consumed",
    "wait_preserves_pending_cloud",
])
def test_actual_fastlio_sync_queue(sync_queue_binary, scenario):
    result = subprocess.run(
        [str(sync_queue_binary), scenario],
        text=True, capture_output=True, timeout=5, check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
