option(INSTALL_PCL "Whether to use system PCL or to install one" ON)

if(${INSTALL_teaserpp})
  set(enable_teaser_dep ON)
else()
  set(enable_teaser_dep OFF)
endif()

if (${INSTALL_PCL})
  find_dependencies(PCL
    DEPENDS Eigen3 Boost flann
  )

  ExternalProject_Add(PCL
    DEPENDS ${PCL_DEPENDS}

    GIT_REPOSITORY https://gitlab.kitware.com/LidarView/pcl
    GIT_TAG pcl-1.13.0-patched
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}
    LOG_INSTALL ON
    PATCH_COMMAND ${pcl_patch_command}

    CMAKE_ARGS
      # Enabled for the SLAM
      -DBUILD_common:BOOL=ON
      -DBUILD_geometry:BOOL=ON
      -DBUILD_io:BOOL=ON
      -DBUILD_octree:BOOL=ON
      -DPCL_ONLY_CORE_POINT_TYPES:BOOL=ON
      # Enabled for teaser
      -DBUILD_2d:BOOL=${enable_teaser_dep}
      -DBUILD_filters:BOOL=${enable_teaser_dep}
      -DBUILD_features:BOOL=${enable_teaser_dep}
      -DBUILD_kdtree:BOOL=${enable_teaser_dep}
      -DBUILD_sample_consensus:BOOL=${enable_teaser_dep}
      -DBUILD_search:BOOL=${enable_teaser_dep}
      # Disabled
      -DBUILD_examples:BOOL=OFF
      -DBUILD_apps:BOOL=OFF
      -DBUILD_benchmarks:BOOL=OFF
      -DBUILD_tools:BOOL=OFF
      -DBUILD_global_tests:BOOL=OFF
      -DBUILD_stereo:BOOL=OFF
      -DBUILD_surface:BOOL=OFF
      -DBUILD_tracking:BOOL=OFF
      -DBUILD_ml:BOOL=OFF
      -DBUILD_keypoints:BOOL=OFF
      -DBUILD_outofcore:BOOL=OFF
      -DBUILD_people:BOOL=OFF
      -DBUILD_CUDA:BOOL=OFF
      -DBUILD_GPU:BOOL=OFF
      -DBUILD_registration:BOOL=OFF
      -DBUILD_recognition:BOOL=OFF
      -DBUILD_segmentation:BOOL=OFF
      -DBUILD_simulation:BOOL=OFF
      -DBUILD_visualization:BOOL=OFF
      -DWITH_CUDA:BOOL=OFF
      -DWITH_QT:BOOL=OFF
      -DWITH_OPENNI=OFF
      -DWITH_OPENNI2=OFF
      -DWITH_VTK:BOOL=OFF
      -DWITH_LIBUSB:BOOL=OFF
      -DWITH_PNG:BOOL=OFF
      -DWITH_QHULL:BOOL=OFF
      -DWITH_RSSDK:BOOL=OFF
      -DWITH_RSSDK2:BOOL=OFF
      -DWITH_OPENGL:BOOL=OFF
      -DBoost_USE_MULTITHREAD:BOOL=ON
      -DBOOST_LIBRARYDIR:PATH=${CMAKE_INSTALL_PREFIX}/lib # The following lines are mandatory to prevent a bug where PCL links against SYSTEM boost
      -DBoost_LIBRARY_DIR_DEBUG:PATH=${CMAKE_INSTALL_PREFIX}/lib
      -DBoost_LIBRARY_DIR_RELEASE:PATH=${CMAKE_INSTALL_PREFIX}/lib
      -DBoost_USE_STATIC_LIBS:BOOL=OFF
      -DBoost_USE_STATIC:BOOL=OFF
      -DPCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32:BOOL=ON

      ${GENERIC_PROJECTS_ARGS}
  )

  reset_patches_step(PCL)
  if (WIN32 AND MSVC AND MSVC_VERSION VERSION_LESS 1910)
    add_patch_command_step(PCL msvc2015-compatibility)
  endif()
endif()
