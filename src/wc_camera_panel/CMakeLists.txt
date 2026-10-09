cmake_minimum_required(VERSION 3.16)
project(wc_camera_panel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Threads REQUIRED)

add_library(wc_camera_preview_core src/decoder.cpp src/frame_store.cpp)
target_include_directories(wc_camera_preview_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wc_camera_preview_core Threads::Threads)
target_compile_options(wc_camera_preview_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(wc_camera_panel SHARED src/camera_panel.cpp src/image_canvas.cpp
  include/wc_camera_panel/camera_panel.hpp)
set_target_properties(wc_camera_panel PROPERTIES AUTOMOC ON)
target_include_directories(wc_camera_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(wc_camera_panel rclcpp sensor_msgs rviz_common pluginlib)
target_link_libraries(wc_camera_panel wc_camera_preview_core Qt5::Widgets Threads::Threads)
target_compile_options(wc_camera_panel PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS wc_camera_preview_core wc_camera_panel
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES ../../config/rviz/cameras.rviz DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  enable_testing()
  add_executable(wc_camera_decoder_tests ../../tests/camera_panel/test_decoder.cpp)
  target_link_libraries(wc_camera_decoder_tests wc_camera_preview_core)
  add_test(NAME wc_camera_decoder COMMAND wc_camera_decoder_tests)
  set_tests_properties(wc_camera_decoder PROPERTIES TIMEOUT 20)

  add_executable(wc_camera_panel_render_tests ../../tests/camera_panel/test_panel_render.cpp)
  ament_target_dependencies(wc_camera_panel_render_tests rviz_common rclcpp sensor_msgs)
  target_link_libraries(wc_camera_panel_render_tests wc_camera_panel Qt5::Widgets)
  add_test(NAME wc_camera_panel_render COMMAND wc_camera_panel_render_tests)
  set_tests_properties(wc_camera_panel_render PROPERTIES TIMEOUT 30 ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endif()
ament_export_include_directories(include)
ament_export_libraries(wc_camera_panel)
ament_export_dependencies(rviz_common rclcpp sensor_msgs pluginlib)
ament_package()
