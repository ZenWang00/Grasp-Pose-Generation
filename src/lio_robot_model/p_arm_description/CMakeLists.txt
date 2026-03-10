cmake_minimum_required(VERSION 3.5)
project(p_arm_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF, meshes, config files
install(DIRECTORY
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

