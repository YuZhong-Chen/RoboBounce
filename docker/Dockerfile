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
    ros-$ROS_DISTRO-ur \
    && rm -rf /var/lib/apt/lists/*

# Change the default user
USER $USERNAME
CMD ["/bin/bash"]
COPY --chown=$USERNAME:$USERNAME \
    .bashrc /home/$USERNAME/.bashrc

# Install custom environment
RUN git clone https://github.com/YuZhong-Chen/.setup_env.git ~/.setup_env \
    && ~/.setup_env/install.sh