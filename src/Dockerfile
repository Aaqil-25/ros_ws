# Use the official ROS Noetic base image
FROM ros:noetic-ros-core

# Set the environment variables to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-rosbag \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
RUN mkdir -p /home/user/ros_ws/src
WORKDIR /home/user/ros_ws

# Copy the package to the workspace
COPY ./rosbag_reader /home/user/ros_ws/src/rosbag_reader

# Copy the .bag file into the container
COPY ./rosbag_reader.bag /home/user/ros_ws/src/rosbag_reader.bag


# Source ROS environment and build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Copy the entrypoint script
COPY ros_entrypoint.sh /

# Make the entrypoint script executable
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
