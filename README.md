# ROS Bag Reader Package

This repository contains a ROS1 package that reads data from a rosbag file and sends it to a server using TCP. 
The package is implemented in C++ and includes a Dockerfile for easy deployment.

## Table of Contents
- [Description](#description)
- [Design Choices](#design-choices)
- [ROS Bag](#ros-bag)
- [Installation](#installation)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Video Demonstration](#video-demonstration)
- [Notes](#notes)

## Description

This package includes:
1. A ROS node (`rosbag_reader`) that subscribes to different topics (GPS, IMU, multiple cameras) in a rosbag file and sends the data to a TCP server.
2. A TCP server node (`tcp_server`) that listens for incoming data from the `rosbag_reader` node.
3. A Dockerfile for containerizing the ROS package.

## Design Choices

### Codebase Structure
- **TcpServer**:
  Implements a basic TCP server that listens for incoming data. By creating a separate server class, ensure that the server logic is encapsulated and can be easily modified or extended.
- **DataSender**:
  This class is solely responsible for managing the connection to the TCP server and sending data. This design allows for easy modifications or replacements of the data transmission method without affecting other parts of the code.
- **rosbag_reader**:
  The ROS node handles the subscription to various topics and processes the incoming data. It uses the `DataSender` class to transmit data, ensuring that the data processing and transmission responsibilities are clearly separated.

## Dependencies

- ros-noetic-ros-base
- ros-noetic-rosbag
- ros-noetic-sensor-msgs
- ros-noetic-std-msgs
- build-essential
- cmake

## Prerequisites

Before proceeding, ensure you have the following installed:
- Docker
- ROS Noetic (if running without Docker)
  
## ROS Bag

Download the rosbag from the following:-
[Google Drive](https://drive.google.com/drive/folders/121qGshjIAAgGuKmm3uYHv3SwXey7lXd3)

**Rosbag file name**: `CA-20190828184706_blur_align.bag`

Place it in the `/home/user/ros_ws/src` directory:

## Installation

1. **Build the Docker image**:
    ```sh
    cd /home/user/ros_ws/src/
    docker build -t rosbag_reader_image .
    ```

2. **Run the Docker container**:
    ```sh
    docker run -it rosbag_reader_image
    ```
   Or directly start the roslaunch file with:
    ```sh
    docker run -it rosbag_reader_image roslaunch rosbag_reader rosbag_reader.launch bag_file:=/home/user/ros_ws/src/rosbag_reader.bag 
    ```
 ps: Don't modify the rosbag file location in the above code.
 
## Usage

1. **Run the launch file to start the nodes**:
   
   please update the file location in the below code by user name.
    ```sh
    roslaunch rosbag_reader rosbag_reader.launch bag_file:=/home/user/ros_ws/src/rosbag_reader.bag
    ```

    This launch file will:
    - Start the `rosbag_play` node to play the specified rosbag file.
    - Start the `tcp_server` node to listen for incoming data.
    - Start the `rosbag_reader` node to read data from the rosbag and send it to the TCP server.

3. **Verify the output**:
    - The TCP server output will be printed in the terminal, showing the received data from the `rosbag_reader` node.

4. **Start `roscore` in a new terminal for node communication**:
    ```sh
    roscore 
    ```

5. **Manually run the separate nodes**:
    - Start the TCP server:
        ```sh
        rosrun rosbag_reader tcp_server
        ```
    - Play the ROSbag:
        ```sh
        rosbag play /home/user/ros_ws/src/rosbag_reader.bag
        ```
    - Start the ROS node:
        ```sh
        rosrun rosbag_reader rosbag_reader
        ```

## Troubleshooting

1. **ROS Environmental Variables Not Set**:
    - Problem: ROS nodes fail to initialize properly due to environmental variables not being set.
    - Solution: Confirm that the ROS environment variables are correctly sourced in your shell session or Docker container. This typically involves sourcing the `setup.bash` script in your ROS installation directory:
        ```sh
        source /opt/ros/<ros_version>/setup.bash
        ```

2. **Permissions Errors During Installation**:
    - Problem: Errors related to permissions during package installation or execution.
    - Solution: Run the installation commands with appropriate permissions. If you experience permission errors when running docker commands, consider using `sudo`:
        ```sh
        sudo docker run
        ```
      Or adjust the file and directory permissions with `chmod` or `chown`:
        ```sh
        chmod 755 script.sh
        ```

## Video Demonstration

A video demonstrating the building and running of the ROS package, including the launching of the nodes from the terminal, is available at:
[Google Drive Video](https://drive.google.com/file/d/1_nqbWMQXQoI0ozgSerSH4irqSETlYjm6/view?usp=drive_link)

**File**: `video/1.mp4`

## Notes

- Ensure that the rosbag file path is correctly specified in the launch file.
- The package assumes the server will be running locally on `127.0.0.1` and port `8000`.

