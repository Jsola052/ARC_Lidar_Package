#!/bin/sh

set -e

apt update

# Install xvfb for offline graphical testing
apt install -y --no-install-recommends \
    xvfb libxcursor1

# Development tools
apt install -y --no-install-recommends \
    clang-tools clang-tidy clang-format cmake cmake-curses-gui \
    gcc g++ ninja-build curl cppcheck git

# Slam dependencies
apt install -y --no-install-recommends \
    libgoogle-glog-dev libeigen3-dev

apt clean all
