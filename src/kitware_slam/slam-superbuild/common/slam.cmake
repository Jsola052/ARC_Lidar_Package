option(USE_CUSTOM_SLAM "Whether to use custom SLAM (default lays in superbuild parent directory) or to download sources" ON)
option(SLAM_PARAVIEW_PLUGIN "Whether to build a SLAM plugin paraview wrapping" OFF)
option(BUILD_SLAM_SHARED_LIBS "Build shared libs on SLAM (FORCE to OFF if paraview wrapping is activated)" ON)
option(ENABLE_OpenCV "Build SLAM parts that require OPENCV if OPENCV is found" OFF)
option(ENABLE_g2o "Build SLAM parts that require G2o if G2o is found" OFF)
option(ENABLE_GTSAM "Build SLAM parts that require Gtsam if Gtsam is found" OFF)
option(ENABLE_teaserpp "Build SLAM parts that require teaserpp if teaserpp is found" OFF)

set(BUILD_SHARED_LIBS ${BUILD_SLAM_SHARED_LIBS})

if (${SLAM_PARAVIEW_PLUGIN})
  set(PARAVIEW_BUILD_PATH "" CACHE PATH "PATH to Paraview build directory")
  # Build LidarSlam lib as STATIC in order to have a standalone LidarSlamPlugin
  set(BUILD_SHARED_LIBS OFF)
endif()

if (${USE_CUSTOM_SLAM})
  get_filename_component(SB_SOURCE_PARENT ${CMAKE_SOURCE_DIR} DIRECTORY)
  set(SLAM_SOURCE_PATH "${SB_SOURCE_PARENT}" CACHE PATH "PATH to SLAM directory")
endif()

find_dependencies(SLAM
  DEPENDS Eigen3 Boost PCL Ceres nanoflann g2o GTSAM teaserpp
)

set(download_step)
if (${USE_CUSTOM_SLAM})
  if (IS_DIRECTORY ${SLAM_SOURCE_PATH})
    set(download_step
      SOURCE_DIR ${SLAM_SOURCE_PATH}
      DOWNLOAD_COMMAND ""
    )
  else()
    message(FATAL_ERROR "SLAM path \"${SLAM_SOURCE_PATH}\" is not a valid directory.")
  endif()
else()
  set(download_step
    GIT_REPOSITORY https://gitlab.kitware.com/keu-computervision/slam.git
    GIT_TAG master
    GIT_SHALLOW 1
  )
endif()

ExternalProject_Add(SLAM
  DEPENDS ${SLAM_DEPENDS}

  ${download_step}
  PREFIX ${PREFIX_DIR}

  CMAKE_ARGS
    -DSLAM_PARAVIEW_PLUGIN:BOOL=${SLAM_PARAVIEW_PLUGIN}
    -DParaView_DIR:PATH=${PARAVIEW_BUILD_PATH}
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DENABLE_OpenCV=${ENABLE_OpenCV}
    -DENABLE_g2o=${ENABLE_g2o}
    -DENABLE_GTSAM=${ENABLE_GTSAM}
    -DENABLE_teaserpp=${ENABLE_teaserpp}
    -DBoost_USE_STATIC_LIBS:BOOL=OFF # Needed and used by PCL only
    ${GENERIC_PROJECTS_ARGS}
)
