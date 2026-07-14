#!/bin/bash
set -e

# setup ros2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# setup slam environment
source /root/slam/install/setup.bash --
# Follow interactively
exec "$@"