option(INSTALL_Boost "Whether to use system boost or to install one" ON)

if (${INSTALL_Boost})
  if (NOT MSVC_VERSION VERSION_GREATER 1800)
    message(FATAL_ERROR "At least Visual Studio 12.0 is required")
  elseif (NOT MSVC_VERSION VERSION_GREATER 1900)
    set(msvc_ver 14.0)
  elseif (NOT MSVC_VERSION VERSION_GREATER 1919)
    set(msvc_ver 14.1)
  elseif (NOT MSVC_VERSION VERSION_GREATER 1930)
    set(msvc_ver 14.2)
  elseif (NOT MSVC_VERSION VERSION_GREATER 1932)
    set(msvc_ver 14.2)
  elseif (NOT MSVC_VERSION VERSION_GREATER 1934)
    set(msvc_ver 14.3)
  else ()
    message(FATAL_ERROR "Unrecognized MSVC version: ${MSVC_VERSION}")
  endif ()

  set(boost_platform_options)
  list(APPEND boost_platform_options "--toolset=msvc-${msvc_ver}" address-model=64)
  list(APPEND boost_platform_options threading=multi)

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
  list(APPEND boost_options --prefix=${CMAKE_INSTALL_PREFIX})
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
      <SOURCE_DIR>/bootstrap.bat
    BUILD_COMMAND
      <SOURCE_DIR>/b2
        ${boost_options}
        ${boost_platform_options}
        install
    INSTALL_COMMAND
      ""
  )

  ExternalProject_Add_Step(Boost Boost-copylibs
    COMMAND "${CMAKE_COMMAND}"
      -Dinstall_location:PATH=${CMAKE_INSTALL_PREFIX}
      -P "${CMAKE_CURRENT_LIST_DIR}/scripts/boost.install.cmake"
    DEPENDEES install
    COMMENT "Copy .dll files to the bin/ directory"
    WORKING_DIRECTORY <SOURCE_DIR>)

  set(Boost_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/Boost-1.76.0)
  set(Boost_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/boost-1_76)


endif()
