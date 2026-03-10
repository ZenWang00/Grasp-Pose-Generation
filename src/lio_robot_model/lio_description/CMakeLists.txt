cmake_minimum_required(VERSION 3.8)
project(lio_description)

find_package(ament_cmake REQUIRED)

# Install launch, rviz, and urdf/xacro files
install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
