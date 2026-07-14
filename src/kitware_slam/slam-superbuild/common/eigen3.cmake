option(INSTALL_Eigen3 "Whether to use system Eigen3 or to install one" ON)

if (${INSTALL_Eigen3})
  ExternalProject_Add(Eigen3

    GIT_REPOSITORY https://gitlab.com/libeigen/eigen
    GIT_TAG 3.4
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}

    CMAKE_ARGS
      -DBUILD_TESTING:BOOL=OFF
      ${GENERIC_PROJECTS_ARGS}
  )
endif()
