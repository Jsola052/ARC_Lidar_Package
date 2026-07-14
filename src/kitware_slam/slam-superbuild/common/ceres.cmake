option(INSTALL_Ceres "Whether to use system Ceres or to install one" ON)

if (${INSTALL_Ceres})
  find_dependencies(Ceres
    DEPENDS Boost Eigen3 glog
  )

  if (WIN32)
    set(extra_cxx_flags "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} /MD")
  endif()

  ExternalProject_Add(Ceres
    DEPENDS ${Ceres_DEPENDS}

    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver
    GIT_TAG 2.2.0 # Warning : this version is not compatible with
                  # old gcc version (e.g. Ubuntu 16 default gcc)
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}

    CMAKE_ARGS
      -DBUILD_EXAMPLES:BOOL=OFF
      -DBUILD_TESTING:BOOL=OFF
      -DUSE_CUDA:BOOL=OFF
      ${GENERIC_PROJECTS_ARGS}
      ${extra_cxx_flags}
  )
endif()
