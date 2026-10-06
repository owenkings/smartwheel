cmake_minimum_required(VERSION 3.8)
project(wc_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SourceFrame.msg" "msg/FusionBundle.msg" "msg/TrackingAssessment.msg"
  "msg/AcceptedBundle.msg"
  "msg/H30Frame.msg"
  "msg/GraphNode.msg" "msg/GraphEdge.msg" "msg/GraphSnapshot.msg"
  "msg/MapStatus.msg" "srv/Resume.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs nav_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
