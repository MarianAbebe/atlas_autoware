cmake_minimum_required(VERSION 3.8)
project(atlas_launch)

find_package(ament_cmake REQUIRED)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
