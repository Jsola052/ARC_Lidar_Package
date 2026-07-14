option(INSTALL_g2o "Whether to use system G2O or to install one" OFF)

if (${INSTALL_g2o})
  find_dependencies(g2o
    DEPENDS Eigen3 Ceres
  )

  ExternalProject_Add(g2o
    DEPENDS ${g2o_DEPENDS}

    GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o
    GIT_TAG 20230806_git
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}

    CMAKE_ARGS
      -DG2O_USE_OPENGL:BOOL=OFF
      -DG2O_USE_VENDORED_CERES:BOOL=OFF
      -DG2O_BUILD_APPS:BOOL=OFF
      -DG2O_BUILD_EXAMPLES:BOOL=OFF
      ${GENERIC_PROJECTS_ARGS}
  )
endif()
