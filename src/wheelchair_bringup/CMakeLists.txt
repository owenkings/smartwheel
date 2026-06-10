cmake_minimum_required(VERSION 3.8)
project(wheelchair_bringup)

find_package(ament_cmake REQUIRED)

add_library(xt_bindshim SHARED src/xt_bindshim.c)
target_link_libraries(xt_bindshim PRIVATE dl)

# RViz teleop panel plugin -------------------------------------------------
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(TELEOP_PANEL_MOC
  include/wheelchair_bringup/teleop_panel.hpp
)

add_library(wheelchair_teleop_panel SHARED
  src/teleop_panel.cpp
  ${TELEOP_PANEL_MOC}
)
target_include_directories(wheelchair_teleop_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wheelchair_teleop_panel
  rviz_common
  rclcpp
  geometry_msgs
  pluginlib
)
target_link_libraries(wheelchair_teleop_panel Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common rviz_panel_plugin.xml)

install(TARGETS wheelchair_teleop_panel
  EXPORT export_wheelchair_teleop_panel
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES rviz_panel_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
# --------------------------------------------------------------------------

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS xt_bindshim
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
