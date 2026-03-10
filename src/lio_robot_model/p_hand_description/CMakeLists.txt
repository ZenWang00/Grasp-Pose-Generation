cmake_minimum_required(VERSION 3.5)
project(p_hand_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

# Install URDFs, meshes, and any launch/config directories
install(DIRECTORY
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

