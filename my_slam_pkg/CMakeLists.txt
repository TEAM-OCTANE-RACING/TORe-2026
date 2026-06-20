cmake_minimum_required(VERSION 3.8)
project(my_slam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(GTSAM REQUIRED)

# Eigen3 is required for the FastSLAM math
find_package(Eigen3 REQUIRED)

# Add the executable
add_executable(eufs_slam_node src/Slam.cpp)

# Target include directories for Eigen
target_include_directories(eufs_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# --- THE FIX: Changed from ekf_slam_node to eufs_slam_node ---
ament_target_dependencies(eufs_slam_node
  rclcpp
  nav_msgs
  geometry_msgs
  eufs_msgs
  tf2
  tf2_ros
  visualization_msgs
)

target_link_libraries(eufs_slam_node gtsam)

install(TARGETS eufs_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
