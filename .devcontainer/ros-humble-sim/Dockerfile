FROM osrf/ros:humble-desktop

# Update the system
RUN apt-get update
RUN apt-get -y full-upgrade
RUN rm -rf /var/lib/apt/lists/*

# Add a default non root user with the same UID and GID of the host user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Give the non root user sudo access
RUN apt-get update
RUN apt-get install -y sudo
RUN rm -rf /var/lib/apt/lists/*
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non root user
USER $USERNAME

# Add the non root user to video group to allow access to webcam
RUN sudo usermod -aG video $USERNAME

# Install essential tools
RUN sudo apt-get update 
RUN sudo apt-get install -y \
    curl \
    git
RUN sudo rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update

# Remove the preinstalled version of Rviz (11.2.19)
# since it introduces incompatibilities with the MoveIt Setup Assistant
RUN sudo apt-get remove -y ros-humble-rviz* --purge

# Install needed ROS packages
RUN sudo apt-get update
RUN sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool
RUN sudo rm -rf /var/lib/apt/lists/*

# Source the ROS setup file
RUN echo "# ROS Humble" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "# colcon_cd" >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
RUN echo "# colcon_argcomplete" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Source the environments
## MoveIt2
RUN echo "# MoveIt2" >> ~/.bashrc
RUN echo "source /ros_workspace/ws_moveit2/install/setup.bash" >> ~/.bashrc
## Rviz2
RUN echo "# Rviz2 11.2.18" >> ~/.bashrc
RUN echo "source /ros_workspace/ws_rviz2/install/setup.bash" >> ~/.bashrc
## Universal Robots
RUN echo "# Universal Robots" >> ~/.bashrc
RUN echo "source /ros_workspace/ws_ur_ext/install/setup.bash" >> ~/.bashrc
## Robotiq Gripper
RUN echo "# Robotiq Gripper" >> ~/.bashrc
RUN echo "source /ros_workspace/ws_robotiq_gripper/install/setup.bash" >> ~/.bashrc
# Project
RUN echo "# Project" >> ~/.bashrc
RUN echo "source /ros_workspace/ws_ur/install/setup.bash" >> ~/.bashrc