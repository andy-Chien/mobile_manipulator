# Copyright 2020-2023 ShaoYu<shaoyu.chien@deltaww.com>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# This Dockerfile is based on https://github.com/AtsushiSaito/docker-ubuntu-sweb
# which is released under the Apache-2.0 license.

FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ARG TARGETPLATFORM
LABEL maintainer="ShaoYu<shaoyu.chien@deltaww.com>"

SHELL ["/bin/bash", "-c"]

# Change source first
RUN sed -i 's/archive.ubuntu.com/free.nchc.org.tw/g' /etc/apt/sources.list

# Upgrade OS
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Add Package
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        tigervnc-standalone-server tigervnc-common \
        supervisor wget curl gosu git sudo python3-pip tini \
        build-essential vim sudo lsb-release locales \
        bash-completion tzdata terminator \
        dos2unix && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# Install ROS
ENV ROS_DISTRO humble
# desktop or ros-base
ARG INSTALL_PACKAGE=desktop

RUN apt-get update -q && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    ros-humble-rmw-cyclonedds-cpp && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

# Install simulation package only on amd64
# Not ready for arm64 for now (July 28th, 2020)
# https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56#issuecomment-1196359860
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
    apt-get update -q && \
    apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-ign && \
    rm -rf /var/lib/apt/lists/*; \
    fi

ARG USER_NAME=ubuntu
ARG USER_PASSWD=ubuntu
RUN useradd --create-home --shell /bin/bash --user-group --groups sudo $USER_NAME
RUN echo "${USER_NAME}:${USER_PASSWD}" | /usr/sbin/chpasswd 2> /dev/null || echo ""

USER $USER_NAME
RUN rosdep update
USER root

# Upgrade OS
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get autoclean && \
    apt-get autoremove

# Enable apt-get completion after running `apt-get update` in the container
RUN rm /etc/apt/apt.conf.d/docker-clean

WORKDIR /home/$USER_NAME/

COPY ./entrypoint.sh /home/$USER_NAME/.entrypoint.sh
COPY ./cyclone_config.xml /home/$USER_NAME/.cyclone_config.xml
RUN dos2unix /home/$USER_NAME/.entrypoint.sh
RUN dos2unix /home/$USER_NAME/.cyclone_config.xml
ENTRYPOINT [ "/bin/tini", "--", "./.entrypoint.sh" ]

USER $USER_NAME

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=99" >> ~/.bashrc
RUN echo "export CYCLONEDDS_URI=file:///absolute/path/to/config_file.xml" >> ~/.bashrc
RUN echo "# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

ARG WS_NAME=hello_ws
RUN mkdir $WS_NAME
ENV USER_NAME $USER_NAME
ENV PASSWORD $USER_PASSWD

