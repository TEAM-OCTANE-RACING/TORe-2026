cmake_minimum_required(VERSION 3.5)
project(my_state_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 1. FIND PACKAGES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

# 2. ADD EXECUTABLE
add_executable(state_estimator_node src/State_Estimator.cpp)

# 3. INCLUDE DIRECTORIES (Matches Step 2)
target_include_directories(state_estimator_node PRIVATE ${Eigen3_INCLUDE_DIRS})
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 4. LINK DEPENDENCIES (Matches Step 2)
ament_target_dependencies(state_estimator_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  eufs_msgs
  tf2_ros
)

# 5. INSTALL TARGETS (Matches Step 2)
install(TARGETS
  state_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
