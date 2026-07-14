option(INSTALL_glog "Whether to use system glog or to install one" ON)

if (${INSTALL_glog})
  ExternalProject_Add(glog

    GIT_REPOSITORY https://github.com/google/glog
    GIT_TAG v0.6.0
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}

    CMAKE_ARGS
      -DWITH_GTEST:BOOL=OFF
      -DWITH_GFLAGS:BOOL=OFF
      ${GENERIC_PROJECTS_ARGS}
  )
endif()
