cmake_minimum_required(VERSION 3.9)
project(lidar_slam VERSION 3.1)

# Compile as C++17, supported in ROS Humble and newer
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "ROS2 Wrapping : No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(BUILD_SLAM_LIB "Build the SLAM library before the ROS2 package" ON)

# Add binobj to increase address capacity on Windows
if (WIN32)
  add_compile_options(/bigobj)
endif()

##################
## Dependencies ##
##################

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
# Generate custom interfaces
find_package(rosidl_default_generators REQUIRED)

# Dependencies of LidarSlam
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(cv_bridge)

###########
## Build ##
###########

# To find shared libraries used by the project at runtime
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# If lib_name has been installed with the superbuild, set lib_name_DIR to install_path
function(SET_SB_LIB lib_name lib_install_path)
  if (EXISTS ${lib_install_path})
    set("${lib_name}_DIR" ${lib_install_path} CACHE PATH "PATH to ${lib_name} directory")
    message(STATUS "${lib_name}_DIR set to ${lib_install_path}")
  else()
    message(STATUS "${lib_name} not found in SuperBuild")
  endif()
endfunction()

# Add path to use superbuild
if (SUPERBUILD_INSTALL_DIR)
  # Set dependencies installed with the superbuild
  SET_SB_LIB(glog "${SUPERBUILD_INSTALL_DIR}/lib/cmake/glog")
  SET_SB_LIB(Eigen3 "${SUPERBUILD_INSTALL_DIR}/share/eigen3/cmake")
  SET_SB_LIB(Ceres "${SUPERBUILD_INSTALL_DIR}/lib/cmake/Ceres")
  SET_SB_LIB(nanoflann "${SUPERBUILD_INSTALL_DIR}/share/nanoflann/cmake")
  SET_SB_LIB(Boost "${SUPERBUILD_INSTALL_DIR}/lib/cmake/Boost-1.76.0")
  if (EXISTS "${SUPERBUILD_INSTALL_DIR}/install/include/boost")
    set(BOOST_ROOT  "${SUPERBUILD_INSTALL_DIR}/install")
    message(STATUS "BOOST_ROOT set to ${SUPERBUILD_INSTALL_DIR}/install")
  else()
    message(STATUS "BOOST_ROOT not found in SuperBuild")
  endif()
  SET_SB_LIB(GTSAM "${SUPERBUILD_INSTALL_DIR}/lib/cmake/GTSAM")
  SET_SB_LIB(g2o "${SUPERBUILD_INSTALL_DIR}/lib/cmake/g2o")
  if (EXISTS "${SUPERBUILD_INSTALL_DIR}/include/pcl-1.10")
    message(FATAL_ERROR "PCL was built within the superbuild : this would cause runtime issues so the process is aborted.
    Rebuild the superbuild from scratch without PCL (`-DINSTALL_PCL=OFF`) and rebuild the ROS package afterwards.")
  endif()
  if (NOT BUILD_SLAM_LIB)
    SET_SB_LIB(LidarSlam "${SUPERBUILD_INSTALL_DIR}/lib/cmake/LidarSlam")
  endif()
endif()


if (BUILD_SLAM_LIB)
  set(slamlib_from_subdirectory true)
  get_filename_component(REAL_CURRENT_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
  get_filename_component(REAL_CORE_DIR "${REAL_CURRENT_DIR}/../.." REALPATH)
  add_subdirectory(${REAL_CORE_DIR} ${CMAKE_BINARY_DIR}/slam)
  set(LidarSlam_target LidarSlam)
else()
  find_package(LidarSlam REQUIRED)
  set(LidarSlam_target LidarSlam::LidarSlam)
endif()

# Generate slam interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Confidence.msg"
  "msg/SlamCommand.msg"
  "srv/SavePc.srv"
  "srv/Reset.srv"
  "srv/SetInitialState.srv"
  "srv/DetectLoop.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Generate C++ code to use custom interfaces in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add LiDAR SLAM ROS node, link dependencies and librairies
add_executable(lidar_slam_node
  src/LidarSlamNode.cxx
  src/LidarSlamNode_main.cxx
)

# Optionnal dependency cv_bridge
if(cv_bridge_FOUND)
  message(STATUS "ROS2 wrapping : cv_bridge was found, camera interface is built")
  list(APPEND optional_DEP cv_bridge)
  list(APPEND optional_LIBRARIES ${cv_bridge_LIBRARIES})
  # Anything else you need to do to enable use of the optional dep, like add definitions
  target_compile_definitions(lidar_slam_node PUBLIC "-DUSE_CV_BRIDGE")
else()
  message(STATUS "ROS wrapping : cv_bridge was not found, camera interface cannot be used")
endif()

set(lidar_slam_dependencies
  rclcpp
  tf2_ros
  pcl_conversions
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  apriltag_ros
  ${optional_DEP}
)

ament_target_dependencies(lidar_slam_node
  ${lidar_slam_dependencies}
)

target_link_libraries(lidar_slam_node
  ${LidarSlam_target}
  "${cpp_typesupport_target}"
  ${optional_LIBRARIES}
  )

# Add aggregation node, link dependencies and librairies
add_executable(aggregation_node
  src/AggregationNode.cxx
)
ament_target_dependencies(aggregation_node
  rclcpp
  pcl_conversions
  std_msgs
  nav_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(aggregation_node
  ${LidarSlam_target}
  "${cpp_typesupport_target}"
)

#############
## Install ##
#############

install(TARGETS lidar_slam_node aggregation_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params
        DESTINATION share/${PROJECT_NAME})

# Installs the package.xml
# Installs config (and possibly target) files for CMake so that it can be found by other packages
ament_export_dependencies(${lidar_slam_dependencies})
ament_package()