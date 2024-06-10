# ROS Bag Reader Package

This repository contains a ROS1 package that reads data from a rosbag file and sends it to a server using TCP. The package is implemented in C++ and includes a Dockerfile for easy deployment.

## Table of Contents
- [Description](#description)
- [Design Choices](#design-choices)
- [Installation](#installation)
- [Usage](#usage)
- [ROS Bag](#ros-bag)
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
  Implements a basic TCP server that listens for incoming data. By creating a separate server class, we ensure that the server logic is encapsulated and can be easily modified or extended.
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

## Installation

1.	Build the Docker image:
   
    docker build -t rosbag_reader_image .

3.	Run the Docker container:
   
    docker run -it rosbag_reader_image

## Usage

1.	Run the launch file to start the nodes:
roslaunch rosbag_reader rosbag_reader.launch
This launch file will:
o	Start the rosbag_play node to play the specified rosbag file.
o	Start the tcp_server node to listen for incoming data.
o	Start the rosbag_reader node to read data from the rosbag and send it to the TCP server.

2.	Verify the output:

o	The TCP server output will be printed in the terminal, showing the received data from the rosbag_reader node.

Manually run the separate nodes,
1.	Start the TCP server:
   
rosrun rosbag_reader tcp_server

3.	Play the ROSbag:  
Rename the rosbag file for your convenience.

rosbag play /home/user/ros_ws/src/rosbag_reader.bag

5.	Start the ROS node:
rosrun rosbag_reader rosbag_reader

## ROS Bag

Download the rosbag from the following:
•	https://drive.google.com/drive/folders/121qGshjIAAgGuKmm3uYHv3SwXey7lXd3

•	Rosbag name: CA-20190828184706_blur_align.bag
Place it in the src directory:

## Video Demonstration
A video demonstrating the building and running of the ROS package, including the launching of the nodes from the terminal, is available at:

•	ros_ws/video/1.mp4

## Notes
•	Ensure that the rosbag file path is correctly specified in the launch file.
•	The package assumes the server will be running locally on 127.0.0.1 and port 8000.




