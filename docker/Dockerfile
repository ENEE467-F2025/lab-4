#########################################
##  Base Image for Gazebo Simulation  ##
#########################################

FROM osrf/ros:jazzy-desktop
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

WORKDIR ${HOME}

ENV DEBIAN_FRONTEND=noninteractive

# Basic setup
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends --allow-unauthenticated \
    autoconf \
    automake \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    g++ \
    git \
    iproute2 \
    iputils-ping \
    libxext-dev \
    libx11-dev \
    make \
    mc \
    mesa-utils \
    nano \
    vim \
    tree \
    pkg-config \
    software-properties-common \
    sudo \
    tmux \
    tzdata \
    xclip \
    x11proto-gl-dev \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-pip \
    screen \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# install VS Code server
RUN curl -fsSL https://code-server.dev/install.sh | sh

# install python extension
RUN code-server --install-extension ms-python.python
COPY extensions/ms-iot.vscode-ros-0.9.6.vsix .
RUN code-server --install-extension ms-iot.vscode-ros-0.9.6.vsix

RUN pip install ipython --break-system-packages && \
    export PATH=/home/robot/.local/bin:$PATH

# Set datetime and timezone correctly
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | tee -a /etc/timezone

ENV DEBIAN_FRONTEND=dialog

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-foxglove-bridge

# defaults
ARG userid=1002
ARG groupid=1002

# setup user env
RUN groupadd -g ${groupid} -o robot
RUN useradd -ms /bin/bash --uid ${userid} -g ${groupid} --groups sudo,video robot \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER robot
WORKDIR /home/robot

# ROS_LOCALHOST_ONLY is deprecated
ENV USER=robot \
    LANG=en_US.UTF-8 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    HOME=/home/robot \
    XDG_RUNTIME_DIR=/run/user/${userid} \
    LIBGL_ALWAYS_SOFTWARE=1 \
    TZ=America/New_York

RUN sudo mkdir -p -m 0700 /run/user/${userid} && \
    sudo chown robot:robot /run/user/${userid}

# ros workspace
RUN source /opt/ros/jazzy/setup.bash \
    && mkdir -p ~/ros2_ws/src

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc && \
    echo "alias clc=clear" >> ~/.bashrc

# Setup tmux config
ADD --chown=robot:robot https://raw.githubusercontent.com/MarylandRoboticsCenter/someConfigs/refs/heads/master/.tmux_K.conf /home/robot/.tmux.conf

ENTRYPOINT ["/bin/bash", "-i"]