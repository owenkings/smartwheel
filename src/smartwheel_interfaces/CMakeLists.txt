cmake_minimum_required(VERSION 3.8)
project(smartwheel_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/MappingStatus.msg"
  "msg/SimMotion.msg"
  "msg/WorkbenchStatus.msg"
  "msg/WheelEncoder.msg"
  "srv/MapTask.srv"
  "srv/MapProductTask.srv"
  "srv/WorkbenchCommand.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
