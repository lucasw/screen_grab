#!/bin/bash

apt-get install -yqq catkin
apt-get install -yqq build-essential

apt-get install -yqq python3-rosdep2
# rosdep init
rosdep update

apt-get install -yqq python3-dynamic-reconfigure libdynamic-reconfigure-config-init-mutex-dev
apt-get install -yqq libimage-transport-dev
apt-get install -yqq libnodeletlib-dev
apt-get install -yqq libx11-dev
apt-get install -yqq libcv-bridge-dev python3-cv-bridge
