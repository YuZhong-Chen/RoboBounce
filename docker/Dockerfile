# Base Image: https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full

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
    unzip \
    vim \
    wget \
    zip \
    python3-pip \
    software-properties-common \
    mesa-utils \
    x11-apps \
    x11-utils \
    libgl1 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    # UR driver
    ros-$ROS_DISTRO-ur \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-hardware-interface-testing \
    # Realsense driver
    ros-$ROS_DISTRO-librealsense2* \
    ros-$ROS_DISTRO-launch-pytest \
    python3-tqdm \
    python3-requests \
    # aruco marker detection
    ros-$ROS_DISTRO-aruco-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    # easy_handeye
    numpy \
    transforms3d

# Add the user to the groups for device access permissions
RUN usermod -aG video $USERNAME && \
    usermod -aG dialout $USERNAME && \
    usermod -aG plugdev $USERNAME

# Change the default user
USER $USERNAME
CMD ["/bin/bash"]
COPY --chown=$USERNAME:$USERNAME \
    .bashrc /home/$USERNAME/.bashrc

# Install custom environment
RUN git clone https://github.com/YuZhong-Chen/.setup_env.git ~/.setup_env \
    && ~/.setup_env/install.sh
