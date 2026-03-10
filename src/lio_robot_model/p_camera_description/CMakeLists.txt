cmake_minimum_required(VERSION 3.5)
project(p_camera_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament
find_package(ament_cmake REQUIRED)

# Install folders: urdf, meshes, config, launch (if any)
install(DIRECTORY
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

