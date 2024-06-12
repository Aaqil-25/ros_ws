#!/bin/bash
set -e
# Source the ROS setup.bash file to set up the ROS environment variables
source /opt/ros/noetic/setup.bash
# Source the workspace setup.bash file to overlay the workspace on top of the ROS environment
source /home/user/ros_ws/devel/setup.bash
# Execute the provided command
exec "$@"

