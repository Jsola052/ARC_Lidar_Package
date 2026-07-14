option(INSTALL_teaserpp "Whether to install teaserpp or to use system one" OFF)

if (${INSTALL_teaserpp})
  find_dependencies(teaserpp
    DEPENDS Eigen3 Boost
  )

  ExternalProject_Add(teaserpp
    DEPENDS ${teaserpp_DEPENDS}

    GIT_REPOSITORY https://github.com/MIT-SPARK/TEASER-plusplus.git
    GIT_TAG v2.0
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}

    CMAKE_ARGS
        -DBUILD_TEASER_FPFH:BOOL=OFF
        -DBUILD_TESTS:BOOL=OFF
        -DBUILD_MATLAB_BINDINGS:BOOL=OFF
        -DBUILD_PYTHON_BINDINGS:BOOL=OFF
        -DBUILD_DOC:BOOL=OFF
        -DBUILD_WITH_MARCH_NATIVE:BOOL=OFF
        -DBUILD_MKL:BOOL=OFF
        -DBUILD_GMOCK:BOOL=OFF
        -DBUILD_DIAGNOSTIC_PRINT:BOOL=OFF;
        -DBOOST_ROOT:PATH=${CMAKE_INSTALL_PREFIX}
        ${GENERIC_PROJECTS_ARGS}
  )
endif()
