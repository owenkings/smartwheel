// Synthetic mathematics only: no XtSdk instance, startup, device, or ROS node.
// Link against the reviewed production wc_xt_vendor and OpenCV libraries.
#include "cartesianTransform.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}

void require_table(const XinTan::CartesianTransform &transform, size_t pixels,
                   const std::string &context) {
  require(transform.hasparam, context + ": valid calibration did not enable projection");
  require(transform.outputUndistortedPoints.size() == pixels,
          context + ": undistortion table has incorrect dimensions");
  require(transform.outputUndistortedPointsDeled.size() == pixels,
          context + ": corner mask has incorrect dimensions");
  for (const auto &point : transform.outputUndistortedPoints) {
    require(std::isfinite(point.x) && std::isfinite(point.y),
            context + ": valid calibration generated a non-finite ray");
  }
}
}  // namespace

int main() {
  try {
    std::string tag = "wc_synthetic_cartesian_recovery";
    XinTan::CartesianTransform transform(tag);
    transform.bisM60 = true;
    XinTan::CamParameterS good{};
    good.fx = 80.0f;
    good.fy = 82.0f;
    good.cx = 80.0f;
    good.cy = 30.0f;
    transform.maptable(good);
    require_table(transform, 160 * 60, "initial valid calibration");
    const auto original_table = transform.outputUndistortedPoints;
    transform.maptable(good);
    require_table(transform, 160 * 60, "repeated valid calibration");
    require(transform.outputUndistortedPoints == original_table,
            "repeated valid calibration changed the ray table");

    using Field = float XinTan::CamParameterS::*;
    const Field fields[] = {
        &XinTan::CamParameterS::fx, &XinTan::CamParameterS::fy,
        &XinTan::CamParameterS::cx, &XinTan::CamParameterS::cy,
        &XinTan::CamParameterS::k1, &XinTan::CamParameterS::k2,
        &XinTan::CamParameterS::k3, &XinTan::CamParameterS::p1,
        &XinTan::CamParameterS::p2};
    unsigned recovered = 0;
    for (size_t index = 0; index < sizeof(fields) / sizeof(fields[0]); ++index) {
      for (float value : {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity()}) {
        auto bad = good;
        bad.*fields[index] = value;
        transform.maptable(bad);
        require(!transform.hasparam, "non-finite calibration was accepted");
        std::cout << "rejected non-finite field " << index
                  << "; retrying identical valid calibration" << std::endl;
        transform.maptable(good);
        require_table(transform, 160 * 60,
                      "same valid calibration after non-finite field " + std::to_string(index));
        require(transform.outputUndistortedPoints == original_table,
                "recovery changed the original calibrated rays");
        ++recovered;
      }
    }
    for (Field focal_length : {fields[0], fields[1]}) {
      for (float value : {0.0f, -1.0f}) {
        auto bad = good;
        bad.*focal_length = value;
        transform.maptable(bad);
        require(!transform.hasparam, "non-positive focal length was accepted");
        transform.maptable(good);
        require_table(transform, 160 * 60, "same valid calibration after invalid focal length");
        require(transform.outputUndistortedPoints == original_table,
                "focal-length recovery changed calibrated rays");
        ++recovered;
      }
    }

    // Equality with constructor seed values is not evidence that a table exists.
    XinTan::CartesianTransform first_default(tag);
    auto initial_values = first_default.camParams;
    require(!first_default.hasparam && first_default.outputUndistortedPoints.empty(),
            "constructor unexpectedly claims a calibrated table");
    first_default.maptable(initial_values);
    require_table(first_default, 320 * 240, "first calibration equals constructor seed");

    std::cout << "PASS: " << recovered
              << " invalid-to-same-valid recoveries, stable rays, and first-use cache equality; no hardware\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
