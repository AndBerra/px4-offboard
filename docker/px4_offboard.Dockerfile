ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive

# Install basic dep
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-rclpy \
    bash-completion \
    locales \
    curl \
    wget \
    git \
    nano \
    lsb-release \
    python3-rosdep \
    python3-colcon-common-extensions \
    sudo \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8
    
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
&& sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' ~/.bashrc

# Create ws and clone dep
WORKDIR /ros2_ws/src
RUN git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
RUN git clone --branch release/1.16 https://github.com/PX4/px4_msgs.git


# install dep and build
WORKDIR /ros2_ws
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro $ROS_DISTRO 
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

# source 
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# default cmd 
CMD ["bash"]
