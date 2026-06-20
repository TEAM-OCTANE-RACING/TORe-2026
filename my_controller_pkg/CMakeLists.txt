cmake_minimum_required(VERSION 3.8)
project(my_controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CRITICAL FIX: Force C++17 for std::clamp to work
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find core and controller packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Mission Manager packages
find_package(eufs_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 1. Build the Pure Pursuit Controller
add_executable(pure_pursuit_node src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_node
  rclcpp
  nav_msgs
  ackermann_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
)

# 2. Build the Mission Manager
add_executable(mission_manager src/mission_manager.cpp)
ament_target_dependencies(mission_manager
  rclcpp
  eufs_msgs
  std_srvs
)

# Install BOTH nodes
install(TARGETS 
  pure_pursuit_node
  mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
