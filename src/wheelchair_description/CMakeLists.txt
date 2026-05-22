cmake_minimum_required(VERSION 3.8)
project(wheelchair_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
