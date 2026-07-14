cmake_minimum_required(VERSION 3.9)
project(livox_ros_driver VERSION 3.1)

# Compile as C++17, supported in ROS Humble and newer
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

set(MSG_FILES
  "msg/CustomMsg.msg"
  "msg/CustomPoint.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()