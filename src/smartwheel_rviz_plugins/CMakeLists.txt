cmake_minimum_required(VERSION 3.8)
project(smartwheel_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(smartwheel_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(panel_headers
  include/smartwheel_rviz_plugins/camera_panel.hpp
  include/smartwheel_rviz_plugins/map_products_panel.hpp
  include/smartwheel_rviz_plugins/mapping_control_panel.hpp
  include/smartwheel_rviz_plugins/occupancy_grid_panel.hpp
  include/smartwheel_rviz_plugins/system_status_panel.hpp
  include/smartwheel_rviz_plugins/teleop_panel.hpp
)

add_library(${PROJECT_NAME} SHARED
  ${panel_headers}
  src/camera_panel.cpp
  src/diagnostic_model.cpp
  src/map_products_panel.cpp
  src/mapping_control_panel.cpp
  src/occupancy_grid_model.cpp
  src/occupancy_grid_panel.cpp
  src/plugin_exports.cpp
  src/system_status_panel.cpp
  src/teleop_model.cpp
  src/teleop_panel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge diagnostic_msgs geometry_msgs image_transport nav_msgs pluginlib
  rclcpp rviz_common rviz_default_plugins rviz_rendering sensor_msgs
  smartwheel_interfaces std_msgs std_srvs
)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)
target_compile_definitions(${PROJECT_NAME} PRIVATE SMARTWHEEL_RVIZ_PLUGINS_BUILDING_LIBRARY)

add_executable(mock_safety_supervisor src/mock_safety_supervisor.cpp)
target_include_directories(mock_safety_supervisor PRIVATE include)
ament_target_dependencies(mock_safety_supervisor diagnostic_msgs geometry_msgs rclcpp std_msgs)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mock_safety_supervisor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Test)

  ament_add_gtest(test_models
    test/test_models.cpp
    src/diagnostic_model.cpp
    src/occupancy_grid_model.cpp
    src/teleop_model.cpp
  )
  if(TARGET test_models)
    target_include_directories(test_models PRIVATE include)
    ament_target_dependencies(test_models diagnostic_msgs nav_msgs)
    target_link_libraries(test_models Qt5::Core Qt5::Gui Qt5::Test)
  endif()

  ament_add_pytest_test(test_plugin_description test/test_plugin_description.py)

  ament_add_gtest(test_camera_model test/test_camera_model.cpp)
  if(TARGET test_camera_model)
    target_include_directories(test_camera_model PRIVATE include)
    target_link_libraries(test_camera_model ${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)
    ament_target_dependencies(test_camera_model rclcpp rviz_common)
  endif()

  ament_add_gtest_executable(test_panel_config
    test/test_panel_config.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  if(TARGET test_panel_config)
    target_include_directories(test_panel_config PRIVATE include)
    target_link_libraries(test_panel_config ${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Test ${GTEST_LIBRARIES})
    ament_target_dependencies(test_panel_config rclcpp rviz_common)
    ament_add_gtest_test(test_panel_config ENV "QT_QPA_PLATFORM=offscreen")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rviz_common pluginlib)
ament_package()
