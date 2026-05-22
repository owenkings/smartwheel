cmake_minimum_required(VERSION 3.8)
project(wheelchair_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
