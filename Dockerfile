FROM ros:humble-ros-base AS base
# FROM ros:humble-ros-base-jammy AS base

SHELL ["/bin/bash", "-c"]

# Install key dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
    apt-get -y --quiet --no-install-recommends install \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-"$ROS_DISTRO"-can-msgs \
    ros-"$ROS_DISTRO"-sensor-msgs \
    ros-"$ROS_DISTRO"-dbw-ford-msgs \
    ros-"$ROS_DISTRO"-gps-msgs \
    ros-"$ROS_DISTRO"-image-transport \
    ros-"$ROS_DISTRO"-image-transport-plugins \
    ros-"$ROS_DISTRO"-mcap-vendor \
    ros-"$ROS_DISTRO"-nmea-msgs \
    ros-"$ROS_DISTRO"-novatel-gps-msgs \
    ros-"$ROS_DISTRO"-ouster-msgs \
    ros-"$ROS_DISTRO"-radar-msgs \
    ros-"$ROS_DISTRO"-rmw-cyclonedds-cpp \
    ros-"$ROS_DISTRO"-rosbag2-cpp \
    ros-"$ROS_DISTRO"-rosbag2-storage \
    ros-"$ROS_DISTRO"-rosbag2-storage-mcap \
    ros-"$ROS_DISTRO"-velodyne-msgs \
    ros-"$ROS_DISTRO"-cv-bridge \
    ros-"$ROS_DISTRO"-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folder
ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS

# Set cyclone DDS ROS RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY ./cyclone_dds.xml $ROS_WS/

# Configure Cyclone cfg file
ENV CYCLONEDDS_URI=file://${ROS_WS}/cyclone_dds.xml

# Enable ROS log colorised output
ENV RCUTILS_COLORIZED_OUTPUT=1

# Add to PATH
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

# Create username
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=rosbag_exporter

RUN groupadd -g $GROUP_ID $USERNAME && \
    useradd -u $USER_ID -g $GROUP_ID -m -l $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers


# Give read/write permissions to the user on the ROS_WS directory
RUN chown -R $USERNAME:$USERNAME $ROS_WS && \
    chmod -R 775 $ROS_WS

COPY entrypoint.sh /entrypoint.sh

# -----------------------------------------------------------------------

FROM base AS prebuilt

WORKDIR $ROS_WS

COPY ./ros2_bag_exporter "$ROS_WS"/src/ros2_bag_exporter

# hadolint ignore=SC1091
RUN source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf ./build ./log

# -----------------------------------------------------------------------

FROM base AS dev

# Add command to docker entrypoint to source newly compiled
#   code when running docker container
RUN sed --in-place --expression \
    "\$isource \"$ROS_WS/install/setup.bash\" " \
    /ros_entrypoint.sh

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
    apt-get -y --quiet --no-install-recommends install \
    # Command-line editor
    nano \
    # Ping network tools
    inetutils-ping \
    # Bash auto-completion for convenience
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Add colcon build alias for convenience
RUN echo 'alias colcon_build="colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash"' >> /etc/bash.bashrc

# Define entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# -----------------------------------------------------------------------

FROM base AS runtime

# Copy artifacts/binaries from prebuilt
COPY --from=prebuilt $ROS_WS/install $ROS_WS/install

# Add command to docker entrypoint to source newly compiled
#   code when running docker container
RUN sed --in-place --expression \
    "\$isource \"$ROS_WS/install/setup.bash\" " \
    /ros_entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
