#include "rosbag_reader/TcpServer.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "tcp_server");
    ros::NodeHandle nh;

    const std::string SERVER_IP = "127.0.0.1";
    const int SERVER_PORT = 8000;

    // Create and start the TCP server
    TcpServer tcpServer(SERVER_IP, SERVER_PORT);
    tcpServer.start();

    // ROS event processing loop
    ros::spin();

    // Stop the TCP server before exiting
    tcpServer.stop();
    return 0;
}
