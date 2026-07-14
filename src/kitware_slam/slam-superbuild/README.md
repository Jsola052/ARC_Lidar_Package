# SLAM-Superbuild

SLAM-Superbuild, is a project to build the SLAM and its dependencies.

## Obtaining the source

To obtain the superbuild source locally, clone this repository using
[Git](https://git-scm.com/).
```
$ git clone --recursive https://gitlab.kitware.com/keu-computervision/slam-superbuild.git
```

## Requirements

The superbuild tries to provide all of its own dependencies, but some tooling
is assumed to be available on the host machine.

- Compiler toolchain
  - GCC 8 or newer
  - Xcode 10 or newer (older is probably supported, but untested)
  - MSVC 19.15 or newer

- Tools
  - ninja (or make) for building

## Projects

The following packages can be built within the SLAM-Superbuild:
- `ceres`
- `eigen3`
- `flann`
- `g2o`
- `glog`
- `gtsam`
- `nanoflann`
- `pcl`
- `teaserpp`
- `boost`

## CMake Variables

- `INSTALL_<LIBRARY>`: if selected the `<LIBRARY>` will be built within the superbuild. If off the superbuild will try to use the system library.
- `USE_CUSTOM_SLAM`: if ON the superbuild will ask the path to your local SLAM repository with `SLAM_SOURCE_PATH` variable.
- `SLAM_PARAVIEW_PLUGIN`: if ON will produce a LidarSlam Plugin Paraview wrapping, a paraview need to be installed on your local machine. The path to the paraview should be set with the `PARAVIEW_BUILD_PATH` variable.
