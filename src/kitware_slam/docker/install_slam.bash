#!/bin/sh
set -e

git clone https://gitlab.kitware.com/keu-computervision/slam slam/src/slam
cd slam
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_g2o=true

rm -rf build log src
chmod -R a+xr install