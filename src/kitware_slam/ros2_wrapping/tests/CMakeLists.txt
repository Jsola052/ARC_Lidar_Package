cmake_minimum_required(VERSION 3.9)
project(lidar_slam_test VERSION 3.1)

# Compile as C++17, supported in ROS Humble and newer
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
# # Generate custom interfaces
# find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
# Custom interfaces
find_package(lidar_slam REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# Add LiDAR SLAM ROS node
add_executable(lidar_slam_test_node
  src/LidarSlamTestNode.cxx
)
ament_target_dependencies(lidar_slam_test_node
  rclcpp
  lidar_slam
  nav_msgs
  tf2_eigen
)

#############
## Install ##
#############

install(TARGETS lidar_slam_test_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME})

# Installs the package.xml
# Installs config (and possibly target) files for CMake so that it can be found by other packages
ament_package()