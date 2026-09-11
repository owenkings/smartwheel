cmake_minimum_required(VERSION 3.16)
project(smartwheel_tf_audit LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

set(TF_AUDIT_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../tf_audit.cpp")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tf_audit.cpp")
  # The isolated remote build directory is intentionally self-contained.
  set(TF_AUDIT_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/tf_audit.cpp")
endif()
add_executable(smartwheel_tf_audit "${TF_AUDIT_SOURCE}")
target_compile_features(smartwheel_tf_audit PRIVATE cxx_std_17)
target_compile_options(smartwheel_tf_audit PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(smartwheel_tf_audit rclcpp tf2_msgs)

# This is intentionally not an ament package: it is configured and built
# directly with CMake in an ignored scratch directory, so no package.xml or
# colcon workspace mutation is required.
