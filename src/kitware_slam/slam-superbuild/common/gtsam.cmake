option(INSTALL_GTSAM "Whether to use system Gtsam or to install one" OFF)

if (${INSTALL_GTSAM})
  find_dependencies(GTSAM
    DEPENDS Eigen3 Boost
  )

  ExternalProject_Add(GTSAM
    DEPENDS ${GTSAM_DEPENDS}

    GIT_REPOSITORY https://gitlab.kitware.com/LidarView/gtsam.git
    GIT_TAG 4.2a9-patched # Warning : this version is not compatible with
                          # old gcc version (e.g. Ubuntu 16 default gcc)
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}
    PATCH_COMMAND ${gtsam_patch_command}

    CMAKE_ARGS
      -DGTSAM_USE_SYSTEM_EIGEN:BOOL=ON
      -DGTSAM_BUILD_TESTS:BOOL=OFF
      -DGTSAM_BUILD_EXAMPLES_ALWAYS:BOOL=OFF
      -DGTSAM_UNSTABLE_BUILD_PYTHON:BOOL=OFF
      -DGTSAM_BUILD_UNSTABLE:BOOL=OFF
      -DGTSAM_INSTALL_MATLAB_TOOLBOX:BOOL=OFF
      -DGTSAM_WITH_TBB:BOOL=OFF
      -DGTSAM_DISABLE_NEW_TIMERS:BOOL=OFF
      # Pb of metis dep at packaging
      # see https://github.com/borglab/gtsam/issues/380 for more details
      # Next param disables metis use
      # Warning : graph optimization might be slower than with metis
      -DGTSAM_SUPPORT_NESTED_DISSECTION=OFF
      ${GENERIC_PROJECTS_ARGS}
  )
endif()
