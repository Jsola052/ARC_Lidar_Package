cmake_minimum_required(VERSION 3.9)
project(lidar_conversions VERSION 3.1)

# Compile as C++17, supported in ROS Humble and newer
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

##################
## Dependencies ##
##################

# Find ament_cmake and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
if (NOT WIN32)
  # pcl_ros is not supported on Windows and not needed
  find_package(pcl_ros REQUIRED)
  set(pcl_ros_target pcl_ros)
endif()
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

###########
## Build ##
###########

# This package needs LidarPoint definition in LidarSlam lib.
# Sadly, adding lidar_slam dependency will create a circular dependency:
# - lidar_conversions needs a header file from LidarSlam, exported with lidar_slam: build dependency
# - lidar_slam uses lidar_conversions in its launchfiles: exec dependency
# These "soft" circular dependencies (build/exec) are not supported by catkin tools.
# To avoid this, we directly include the correct headers.
include_directories(
  "../../slam_lib/include/"
  "./external_include"
)

# Generate slam interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EstimParams.srv"
  "srv/EstimSense.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Generate C++ code to use custom interfaces in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Velodyne Lidar
add_executable(velodyne_conversion_node src/VelodyneToLidarNode.cxx)

set (conversion_dependencies
  rclcpp
  ${pcl_ros_target}
  pcl_conversions
  sensor_msgs
  std_msgs
  std_srvs
)

#link dependencies to target
ament_target_dependencies(velodyne_conversion_node
  ${conversion_dependencies}
)

target_link_libraries(velodyne_conversion_node
  "${cpp_typesupport_target}"
)

# Ouster Lidar
add_executable(ouster_conversion_node src/OusterToLidarNode.cxx)

#link dependencies to target
ament_target_dependencies(ouster_conversion_node
  ${conversion_dependencies}
)

target_link_libraries(ouster_conversion_node
  "${cpp_typesupport_target}"
)

# Robosense RSLidar
add_executable(robosense_conversion_node src/RobosenseToLidarNode.cxx)

#link dependencies to target
ament_target_dependencies(robosense_conversion_node
  ${conversion_dependencies}
)

target_link_libraries(robosense_conversion_node
  "${cpp_typesupport_target}"
)

# Livox mid100 Lidar
add_executable(livox_conversion_node src/LivoxToLidarNode.cxx)

#link dependencies to target
ament_target_dependencies(livox_conversion_node
  ${conversion_dependencies}
  livox_ros_driver
)

# Hesai Lidar
add_executable(hesai_conversion_node src/HesaiToLidarNode.cxx)

#link dependencies to target
ament_target_dependencies(hesai_conversion_node
  ${conversion_dependencies}
)

target_link_libraries(hesai_conversion_node
  "${cpp_typesupport_target}"
)

# Generic converter
add_executable(generic_conversion_node src/GenericConversionNode.cxx)

#link dependencies to target
ament_target_dependencies(generic_conversion_node
  ${conversion_dependencies}
)

target_link_libraries(generic_conversion_node
  "${cpp_typesupport_target}"
)

#############
## Install ##
#############

install(TARGETS
    velodyne_conversion_node
    ouster_conversion_node
    robosense_conversion_node
    livox_conversion_node
    hesai_conversion_node
    generic_conversion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${conversion_dependencies})
ament_package()