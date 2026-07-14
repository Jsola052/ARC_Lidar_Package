option(INSTALL_Boost "Whether to use system boost or to install one" ON)

if (${INSTALL_Boost})

  set(boost_required_lib
    # The following are for pcl
    system
    filesystem
    program_options
    # The following are for gtsam
    serialization
    timer
    regex
    # The following are for liblas
    thread
    iostreams
    chrono
    date_time
    atomic)

  set(boost_options ${boost_required_lib})
  list(TRANSFORM boost_options PREPEND --with-)
  list(APPEND boost_options
    --prefix=${CMAKE_INSTALL_PREFIX}
  )

  list(APPEND boost_options link=shared)

  ExternalProject_Add(Boost

    GIT_REPOSITORY https://github.com/boostorg/boost.git
    GIT_TAG boost-1.76.0
    GIT_SHALLOW 1
    PREFIX ${PREFIX_DIR}
    LOG_DOWNLOAD ON
    LOG_INSTALL ON

    BUILD_IN_SOURCE True # needed as Boost is meant to be built in source

    CONFIGURE_COMMAND
      <SOURCE_DIR>/bootstrap.sh --prefix=${CMAKE_INSTALL_PREFIX}
    BUILD_COMMAND
      <SOURCE_DIR>/b2 ${boost_options}
    INSTALL_COMMAND
      <SOURCE_DIR>/b2 ${boost_options} install
  )

  set(Boost_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/Boost-1.76.0)
  set(Boost_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)

endif()