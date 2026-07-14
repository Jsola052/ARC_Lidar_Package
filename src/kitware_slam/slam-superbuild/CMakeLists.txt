cmake_minimum_required(VERSION 3.21)
project(SLAM_Superbuild)

# -----------------------------------------------------------------------------------------------
# Read CMAKE Arguments

# -- Set c++ standard
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "MinSizeRel" "RelWithDebInfo" "Release")
endif()

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "install prefix" FORCE)
    message(STATUS "No install directory specified, set it to ${CMAKE_INSTALL_PREFIX} as default")
endif ()

set(PREFIX_DIR superbuild)

set(CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH}")

set(GENERIC_PROJECTS_ARGS
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
  -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
  -DCMAKE_PREFIX_PATH:STRING=${CMAKE_PREFIX_PATH}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_INSTALL_RPATH:PATH=${CMAKE_INSTALL_PREFIX}/lib/
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=ON
  -DBUILD_SHARED_LIBS:BOOL=ON
  -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
  -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
)

include(ExternalProject)
include(cmake/FindDependencies.cmake)
include(cmake/AddPatchCommandStep.cmake)

# To allow big path on windows
if (WIN32)
  add_compile_options("/bigobj")
endif()

# -----------------------------------------------------------------------------------------------
# INSTALL Eigen3
include(common/eigen3.cmake)

# -----------------------------------------------------------------------------------------------
# INSTALL PCL

# INSTALL Boost (For PCL)
if (WIN32)
  include(win/boost.cmake)
else()
  include(unix/boost.cmake)
endif()

# INSTALL Flann (For PCL)
include(common/flann.cmake)

# INSTALL PCL
include(common/pcl.cmake)

# -----------------------------------------------------------------------------------------------
# INSTALL Nanoflann
include(common/nanoflann.cmake)

# -----------------------------------------------------------------------------------------------
# INSTALL Ceres

# INSTALL Glog (For Ceres)
include(common/glog.cmake)

# INSTALL Ceres
include(common/ceres.cmake)

# -----------------------------------------------------------------------------------------------
# INSTALL G2o
include(common/g2o.cmake)

# -----------------------------------------------------------------------------------------------
# INSTALL Gtsam
include(common/gtsam.cmake)

# -----------------------------------------------------------------------------------------------
# INSTALL Teaserpp
if (WIN32)
  message(WARNING "TEASER++ installation is not supported on windows.")
else()
  include(unix/teaserpp.cmake)
endif()

# -----------------------------------------------------------------------------------------------
# INSTALL SLAM
include(common/slam.cmake)
