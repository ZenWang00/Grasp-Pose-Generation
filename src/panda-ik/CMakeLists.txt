cmake_minimum_required(VERSION 3.8)
project(panda_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# -------------------------------------
# Dependencies
# -------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(geosacs_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# -------------------------------------
# Build Rust library with Cargo
# -------------------------------------
add_custom_target(
  panda_ik_rust ALL
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building Rust library with Cargo"
)

set(RUST_LIB_NAME panda_ik)
set(RUST_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/target/release)
set(RUST_LIB ${RUST_LIB_PATH}/lib${RUST_LIB_NAME}.so)

add_library(${RUST_LIB_NAME}_lib SHARED IMPORTED GLOBAL)
set_target_properties(${RUST_LIB_NAME}_lib PROPERTIES
  IMPORTED_LOCATION ${RUST_LIB}
)

# -------------------------------------
# C++ Executable
# -------------------------------------
add_executable(${PROJECT_NAME}
  cpp_src/main.cpp
)

add_dependencies(${PROJECT_NAME} panda_ik_rust)

target_include_directories(${PROJECT_NAME}
  PUBLIC include
  SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${RUST_LIB_NAME}_lib
)

# Embed rpath to avoid LD_LIBRARY_PATH
target_link_options(${PROJECT_NAME}
  PRIVATE
  "-Wl,-rpath,${CMAKE_INSTALL_PREFIX}/lib"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  geosacs_msgs
)




# -------------------------------------
# C++ Executable: panda_ik_teleop (main_teleop.cpp)
# -------------------------------------
add_executable(panda_ik_teleop
  cpp_src/main_teleop.cpp
)

add_dependencies(panda_ik_teleop panda_ik_rust)

target_include_directories(panda_ik_teleop
  PUBLIC include
  SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(panda_ik_teleop
  ${RUST_LIB_NAME}_lib
)

target_link_options(panda_ik_teleop
  PRIVATE
  "-Wl,-rpath,${CMAKE_INSTALL_PREFIX}/lib"
)

ament_target_dependencies(panda_ik_teleop
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  geosacs_msgs
)

# -------------------------------------
# Install targets and resources
# -------------------------------------
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS panda_ik_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdfs
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  ${RUST_LIB}
  DESTINATION lib
)


# -------------------------------------
# Package definition
# -------------------------------------
ament_package()
