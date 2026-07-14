option(INSTALL_flann "Whether to use system flann or to install one" OFF)

# Flann official version has a cmake issue for windows
# (https://stackoverflow.com/questions/50763621/building-flann-with-cmake-fails)
# We use a patched version
if (${INSTALL_flann})
  ExternalProject_Add(flann
    GIT_REPOSITORY https://gitlab.kitware.com/gabriel.devillers/flann.git
    GIT_TAG 1.9.1-patched
    PREFIX ${PREFIX_DIR}

    CMAKE_ARGS
      -DCMAKE_CXX_STANDARD:STRING=14
      -DBUILD_EXAMPLES:BOOL=OFF
      -DBUILD_DOC:BOOL=OFF
      -DBUILD_TESTS:BOOL=OFF
      -DBUILD_PYTHON_BINDINGS:BOOL=OFF
      -DBUILD_C_BINDINGS:BOOL=OFF
      -DBUILD_MATLAB_BINDINGS:BOOL=OFF
      ${GENERIC_PROJECTS_ARGS}
  )
endif()
