cmake_minimum_required(VERSION 3.16)
project(wc_xt_driver VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(wc_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem date_time regex)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(XT_VENDOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../SDKs/xtsdk_ros/xtsdk_cpp/xtsdk" CACHE PATH "Reviewed project-local XT vendor source")
get_filename_component(XT_VENDOR_ROOT "${XT_VENDOR_ROOT}" REALPATH)
if(NOT EXISTS "${XT_VENDOR_ROOT}/xtsdk.cpp")
  message(FATAL_ERROR "Reviewed SDK source missing at ${XT_VENDOR_ROOT}")
endif()
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|x86_64)$")
  message(FATAL_ERROR "Only reviewed Linux aarch64/x86_64 SDK binary is available")
endif()
set(XT_FILTER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../SDKs/xtsdk_filter_3d3db067" CACHE PATH "Pinned matching spatial filter ABI bundle")
get_filename_component(XT_FILTER_ROOT "${XT_FILTER_ROOT}" REALPATH)
set(XT_LIBRARY_DIR "${XT_FILTER_ROOT}/lib/linux/${CMAKE_SYSTEM_PROCESSOR}")
set(WC_FILTER_MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor_patches/xtsdk_filter_3d3db067.manifest.json")
# The official filter .so leaves legacy TBB task symbols undefined without a
# DT_NEEDED entry. The unversioned libtbb.so on Orin is oneTBB (SONAME 12), which
# cannot satisfy that ABI. Select the exact existing SONAME 2, never install it.
find_library(WC_XT_LEGACY_TBB NAMES libtbb.so.2
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../SDKs/ubuntu_opencv45/sysroot/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
  PATHS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}" "/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
if(NOT WC_XT_LEGACY_TBB)
  message(FATAL_ERROR "Pinned filter requires existing legacy libtbb.so.2; unversioned oneTBB is not a substitute")
endif()
get_filename_component(WC_XT_LEGACY_TBB "${WC_XT_LEGACY_TBB}" REALPATH)
file(SHA256 "${WC_XT_LEGACY_TBB}" WC_XT_LEGACY_TBB_SHA256)

if(NOT EXISTS "${XT_LIBRARY_DIR}/libxtsdk_shared.so")
  message(FATAL_ERROR "Vendor filter binary missing")
endif()
# Use an independent target/build path. Do not execute vendor selros/install
# scripts or its CMake that writes artifacts back into the source directory.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/version.h" "#pragma once\n#define APP_NAME \"XTSDK-reviewed-phase1\"\n#define VERSION_MAJOR 2\n#define VERSION_MINOR 1\n#define VERSION_PATCH 7\n#define VERSION_STRING \"2.1.7+wc-patch\"\n#define BUILD_DATE \"2026-09-11\"\n")
set(XT_SOURCES xtsdk.cpp xtdaemon.cpp communicationNet.cpp communicationUsb.cpp frame.cpp
    cartesianTransform.cpp baseFilter.cpp utils.cpp xtlogger.cpp xtsdkLibHandler.cpp xtpcl_filter.cpp)
list(TRANSFORM XT_SOURCES PREPEND "${XT_VENDOR_ROOT}/")
set(WC_VENDOR_MANIFEST "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor_patches/xtsdk_ros_965d31a.manifest.json")
set(WC_VENDOR_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor_patches/xtsdk_ros_965d31a.patch")
set(WC_VENDOR_CHECKER "${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_vendor.py")
set(WC_PROJECT_INPUTS "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt" "${WC_VENDOR_CHECKER}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/xt_source_node.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/owned_stop.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/acquisition_budget.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/host_lock.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/bounded_work_queue.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/shutdown_deadline.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/device_config_policy.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/imaging_allowlist.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/xtcfg.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/wc_xt_driver/udp_assembler.hpp" "${CMAKE_CURRENT_BINARY_DIR}/version.h")
set(WC_PROJECT_FILE_ARGS)
foreach(input IN LISTS WC_PROJECT_INPUTS)
  list(APPEND WC_PROJECT_FILE_ARGS --project-file "${input}")
endforeach()
execute_process(COMMAND "${Python3_EXECUTABLE}" "${WC_VENDOR_CHECKER}"
  --sdk-root "${XT_VENDOR_ROOT}" --manifest "${WC_VENDOR_MANIFEST}" --patch "${WC_VENDOR_PATCH}"
  --output "${CMAKE_CURRENT_BINARY_DIR}/vendor_provenance.json"
  --filter-root "${XT_FILTER_ROOT}" --filter-manifest "${WC_FILTER_MANIFEST}"
  --temporary-root "${CMAKE_CURRENT_BINARY_DIR}"
  --build-setting "compiler=${CMAKE_CXX_COMPILER}" --build-setting "compiler_version=${CMAKE_CXX_COMPILER_VERSION}"
  --build-setting "standard=${CMAKE_CXX_STANDARD}" --build-setting "build_type=${CMAKE_BUILD_TYPE}"
  --build-setting "cxx_flags=${CMAKE_CXX_FLAGS}" --build-setting "cxx_flags_debug=${CMAKE_CXX_FLAGS_DEBUG}"
  --build-setting "cxx_flags_release=${CMAKE_CXX_FLAGS_RELEASE}" --build-setting "cxx_flags_relwithdebinfo=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}"
  --build-setting "cxx_flags_minsizerel=${CMAKE_CXX_FLAGS_MINSIZEREL}" --build-setting "architecture=${CMAKE_SYSTEM_PROCESSOR}"
  --build-setting "exe_linker_flags=${CMAKE_EXE_LINKER_FLAGS}" --build-setting "shared_linker_flags=${CMAKE_SHARED_LINKER_FLAGS}"
  --build-setting "legacy_tbb_path=${WC_XT_LEGACY_TBB}" --build-setting "legacy_tbb_sha256=${WC_XT_LEGACY_TBB_SHA256}"
  --build-setting "legacy_tbb_link_policy=push-state,no-as-needed,pop-state"
  --build-setting "library_dir=${XT_LIBRARY_DIR}" --build-setting "opencv_includes=${OpenCV_INCLUDE_DIRS}"
  --build-setting "boost_includes=${Boost_INCLUDE_DIRS}" --build-setting "sources=${XT_SOURCES}"
  --build-setting "vendor_definitions=LIB_DIR=${XT_LIBRARY_DIR},PROJECT_DIR=${XT_VENDOR_ROOT}/.."
  ${WC_PROJECT_FILE_ARGS} RESULT_VARIABLE WC_VENDOR_RESULT OUTPUT_VARIABLE WC_VENDOR_OUTPUT ERROR_VARIABLE WC_VENDOR_ERROR)
if(NOT WC_VENDOR_RESULT EQUAL 0)
  message(FATAL_ERROR "Reviewed SDK binding failed; do not skip or overwrite the checkout. ${WC_VENDOR_ERROR}")
endif()
message(STATUS "${WC_VENDOR_OUTPUT}")
string(REGEX MATCH "XT_VENDOR_VERIFIED ([0-9a-f]+) ([0-9a-f]+) (VERIFIED_GIT_CHECKOUT|VERIFIED_EXPORTED_SNAPSHOT)" WC_VENDOR_MATCH "${WC_VENDOR_OUTPUT}")
if(NOT WC_VENDOR_MATCH)
  message(FATAL_ERROR "Verified SDK checker did not return a commit identity")
endif()
set(WC_SDK_COMMIT "${CMAKE_MATCH_1}")
set(WC_SDK_PROVENANCE_MODE "${CMAKE_MATCH_3}")
# Reconfigure and recheck before rebuilding after any SDK source/header/library
# change, including additions. The check is unconditional when BUILD_TESTING=OFF.
file(GLOB_RECURSE WC_VENDOR_INPUTS CONFIGURE_DEPENDS "${XT_VENDOR_ROOT}/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${WC_VENDOR_INPUTS} ${WC_PROJECT_INPUTS} "${WC_VENDOR_MANIFEST}" "${WC_VENDOR_PATCH}" "${WC_FILTER_MANIFEST}"
  "${XT_FILTER_ROOT}/lib/linux/aarch64/libxtsdk_shared.so" "${XT_FILTER_ROOT}/lib/linux/x86_64/libxtsdk_shared.so" "${WC_XT_LEGACY_TBB}")
add_library(wc_xt_vendor STATIC ${XT_SOURCES})
target_include_directories(wc_xt_vendor PUBLIC "${XT_VENDOR_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}" include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_compile_definitions(wc_xt_vendor PRIVATE LIB_DIR="${XT_LIBRARY_DIR}" PROJECT_DIR="${XT_VENDOR_ROOT}/..")
target_link_libraries(wc_xt_vendor PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} Threads::Threads ${CMAKE_DL_LIBS}
  "-Wl,--push-state,--no-as-needed" "${WC_XT_LEGACY_TBB}" "-Wl,--pop-state")
add_executable(xt_source_node src/xt_source_node.cpp)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor_patches/xtsdk_ros_965d31a.patch" WC_PATCH_SHA)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/src/xt_source_node.cpp" WC_DRIVER_SHA)
file(SHA256 "${WC_VENDOR_MANIFEST}" WC_MANIFEST_SHA)
file(SHA256 "${WC_FILTER_MANIFEST}" WC_FILTER_MANIFEST_SHA)
target_compile_definitions(xt_source_node PRIVATE WC_XT_FILTER_MANIFEST_SHA="${WC_FILTER_MANIFEST_SHA}" WC_XT_PATCH_SHA="${WC_PATCH_SHA}" WC_XT_DRIVER_SHA="${WC_DRIVER_SHA}" WC_XT_MANIFEST_SHA="${WC_MANIFEST_SHA}" WC_XT_SDK_COMMIT="${WC_SDK_COMMIT}" WC_XT_SDK_PROVENANCE_MODE="${WC_SDK_PROVENANCE_MODE}")
target_link_libraries(xt_source_node wc_xt_vendor OpenSSL::Crypto)
ament_target_dependencies(xt_source_node rclcpp sensor_msgs std_msgs std_srvs diagnostic_msgs rcl_interfaces wc_interfaces)
target_compile_options(xt_source_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS xt_source_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/vendor_provenance.json" "${WC_VENDOR_MANIFEST}" "${WC_FILTER_MANIFEST}" DESTINATION share/${PROJECT_NAME}/provenance)
if(BUILD_TESTING)
  enable_testing()
  add_executable(xt_bounded_work_queue_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_bounded_work_queue.cpp")
  target_include_directories(xt_bounded_work_queue_test PRIVATE include)
  target_link_libraries(xt_bounded_work_queue_test Threads::Threads)
  target_compile_options(xt_bounded_work_queue_test PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME xt_bounded_work_queue_test COMMAND xt_bounded_work_queue_test)
  set_tests_properties(xt_bounded_work_queue_test PROPERTIES TIMEOUT 15)
  add_executable(xt_frame_snapshot_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_frame_snapshot.cpp")
  target_link_libraries(xt_frame_snapshot_test wc_xt_vendor)
  target_compile_options(xt_frame_snapshot_test PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME xt_frame_snapshot_test COMMAND xt_frame_snapshot_test)
  add_executable(xt_lens_recovery_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/diagnostics/test_cartesian_transform_recovery.cpp")
  target_link_libraries(xt_lens_recovery_test wc_xt_vendor)
  add_test(NAME xt_lens_recovery_test COMMAND xt_lens_recovery_test)
  add_executable(xt_acquisition_budget_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_acquisition_budget.cpp")
  target_include_directories(xt_acquisition_budget_test PRIVATE include)
  add_test(NAME xt_acquisition_budget_test COMMAND xt_acquisition_budget_test)
  add_executable(xt_device_config_policy_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_device_config_policy.cpp")
  target_include_directories(xt_device_config_policy_test PRIVATE include "${XT_VENDOR_ROOT}")
  add_test(NAME xt_device_config_policy_test COMMAND xt_device_config_policy_test)
  add_executable(xtcfg_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xtcfg.cpp")
  target_include_directories(xtcfg_test PRIVATE include)
  add_test(NAME xtcfg_test COMMAND xtcfg_test "${CMAKE_CURRENT_SOURCE_DIR}/config")
  add_executable(xt_filter_runtime_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_filter_runtime.cpp")
  target_link_libraries(xt_filter_runtime_test wc_xt_vendor)
  add_test(NAME xt_filter_runtime_test COMMAND xt_filter_runtime_test "${CMAKE_CURRENT_SOURCE_DIR}/config")
  add_executable(xt_packet_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_packet.cpp")
  target_include_directories(xt_packet_test PRIVATE include)
  add_test(NAME xt_packet_test COMMAND xt_packet_test)
  add_executable(xt_loopback_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_loopback.cpp")
  target_link_libraries(xt_loopback_test wc_xt_vendor)
  add_test(NAME xt_loopback_test COMMAND xt_loopback_test)
  add_executable(xt_hdr_bounds_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_hdr_bounds.cpp")
  target_link_libraries(xt_hdr_bounds_test wc_xt_vendor)
  add_test(NAME xt_hdr_bounds_test COMMAND xt_hdr_bounds_test)
  add_executable(xt_shutdown_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_shutdown.cpp")
  ament_target_dependencies(xt_shutdown_test rclcpp)
  target_include_directories(xt_shutdown_test PRIVATE include)
  target_link_libraries(xt_shutdown_test Threads::Threads)
  target_compile_options(xt_shutdown_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME xt_shutdown_test COMMAND xt_shutdown_test)
  set_tests_properties(xt_shutdown_test PROPERTIES TIMEOUT 45 ENVIRONMENT "ROS_DOMAIN_ID=187;ROS_LOCALHOST_ONLY=1")
  add_executable(xt_owned_stop_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/sensors/test_xt_owned_stop.cpp")
  target_include_directories(xt_owned_stop_test PRIVATE include)
  add_test(NAME xt_owned_stop_test COMMAND xt_owned_stop_test)
endif()
ament_package()
