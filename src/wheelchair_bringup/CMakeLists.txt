cmake_minimum_required(VERSION 3.8)
project(wheelchair_bringup)

find_package(ament_cmake REQUIRED)

add_library(xt_bindshim SHARED src/xt_bindshim.c)
target_link_libraries(xt_bindshim PRIVATE dl)

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
