# Base Image: https://hub.docker.com/r/nvidia/cuda/tags
FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/*
# Add the user to the groups for device access permissions
RUN usermod -aG video $USERNAME && \
    usermod -aG dialout $USERNAME && \
    usermod -aG plugdev $USERNAME
ENV SHELL=/bin/bash
ENV PATH="/home/$USERNAME/.local/bin:${PATH}"

# Install common tools
RUN apt-get update && apt-get install -y \
    curl \
    git \
    htop \
    nvtop \
    iputils-ping \
    nano \
    net-tools \
    tmux \
    zip \
    unzip \
    vim \
    wget \
    mesa-utils \
    x11-apps \
    x11-utils \
    libgl1 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2-humble
# Reference: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
ENV ROS_DISTRO=humble
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    locales \
    software-properties-common \
    tzdata \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init \
    && rosdep update

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    # Install Gazebo and RViz
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    # UR driver
    ros-$ROS_DISTRO-ur \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-hardware-interface-testing \
    ros-$ROS_DISTRO-ros2-control \
    # Realsense driver
    ros-$ROS_DISTRO-librealsense2* \
    ros-$ROS_DISTRO-launch-pytest \
    python3-tqdm \
    python3-requests \
    # aruco marker detection
    ros-$ROS_DISTRO-aruco-ros \
    && rm -rf /var/lib/apt/lists/*

# Install torch and torchvision with CUDA support
RUN pip3 install --index-url https://download.pytorch.org/whl/cu129 \
    torch \
    torchvision

# Install Python packages
RUN pip3 install \
    # easy_handeye
    transforms3d \
    # Yolo
    ultralytics \
    numpy==1.24.4

# Change the default user
USER $USERNAME
CMD ["/bin/bash"]
COPY --chown=$USERNAME:$USERNAME \
    .bashrc /home/$USERNAME/.bashrc

# Install custom environment
RUN git clone https://github.com/YuZhong-Chen/.setup_env.git ~/.setup_env \
    && ~/.setup_env/install.sh
