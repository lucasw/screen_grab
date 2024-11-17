# docker build . -t ubuntu_2204_screen_grab
ARG IMAGE=ubuntu:22.04
FROM ${IMAGE}
ARG IMAGE
RUN echo ${IMAGE}

ENV DEBIAN_FRONTEND="noninteractive"

# be able to source files
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update -yqq
RUN apt-get upgrade -yqq
RUN apt-get install -yqq apt-utils

RUN apt-get install -yqq catkin
RUN apt-get install -yqq build-essential
RUN apt-get install -yqq python3-rosdep2

# RUN rosdep init
RUN rosdep update
# RUN ROS_DISTRO=one rosdep install --from-paths src --ignore-src -r -s  # do a dry-run first
# RUN ROS_DISTRO=one rosdep install --from-paths src --ignore-src -r -y
# RUN catkin_make
RUN apt-get install -yqq python3-dynamic-reconfigure libdynamic-reconfigure-config-init-mutex-dev
RUN apt-get update -yqq && apt-get install -yqq libimage-transport-dev
RUN apt-get install -yqq libnodeletlib-dev
RUN apt-get install -yqq libx11-dev
RUN apt-get install -yqq libcv-bridge-dev python3-cv-bridge
RUN apt-get install -yqq git

WORKDIR /home/catkin_ws/src
RUN git clone https://github.com/lucasw/roslint
COPY screen_grab /home/catkin_ws/src/screen_grab
WORKDIR /home/catkin_ws
RUN catkin_make
RUN catkin_make screen_grab --cmake-args roslint
