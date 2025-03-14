#!/bin/bash
set -e

# Setup ROS2 env
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Source ros_ws if it was built
if [ -f /opt/ros_ws/install/setup.bash ]; then
    source /opt/ros_ws/install/setup.bash
fi


# If no command is provided, start an interactive bash session
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi
