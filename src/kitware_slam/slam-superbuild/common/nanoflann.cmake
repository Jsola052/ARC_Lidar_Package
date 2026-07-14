option(INSTALL_nanoflann "Whether to use system Nanoflann or to install one" ON)

if (${INSTALL_nanoflann})
  find_dependencies(nanoflann
    DEPENDS Eigen3
  )

  ExternalProject_Add(nanoflann
    DEPENDS ${nanoflann_DEPENDS}

    GIT_REPOSITORY https://github.com/jlblancoc/nanoflann
    GIT_TAG v1.5.4
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}

    CMAKE_ARGS
      -DNANOFLANN_BUILD_TESTS:BOOL=OFF
      -DNANOFLANN_BUILD_EXAMPLES:BOOL=OFF
      ${GENERIC_PROJECTS_ARGS}
  )
endif()
