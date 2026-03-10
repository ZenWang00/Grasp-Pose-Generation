cmake_minimum_required(VERSION 3.5)
project(p_grip_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the meshes, URDFs, and launch files
install(DIRECTORY
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

