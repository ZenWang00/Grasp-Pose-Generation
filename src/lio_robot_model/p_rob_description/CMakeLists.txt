cmake_minimum_required(VERSION 3.8)
project(p_rob_description)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

# Install important folders like urdf, meshes, launch files, etc.
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
)

ament_package()

