#include "rosbag_reader/DataSender.h"
#include <chrono>
#include <thread>

DataSender::DataSender(const std::string &server_ip, int server_port)
    : server_ip_(server_ip), server_port_(server_port), connected_(false) {
    connectToServer();
}

DataSender::~DataSender() {
        close(sock);
}

void DataSender::connectToServer() {
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_ERROR("Socket creation error");
        exit(EXIT_FAILURE);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(server_port_);

    if (inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
        ROS_ERROR("Invalid address/ Address not supported");
        exit(EXIT_FAILURE);
    }

    // Retry connecting to the server
    while (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_WARN("Connection Failed. Retrying in 1 second...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    connected_ = true;
    ROS_INFO("Connected to server %s:%d", server_ip_.c_str(), server_port_);
}

void DataSender::sendData(const std::string &data) {
    if (!connected_) {
        ROS_WARN("Not connected to server. Trying to reconnect...");
        connectToServer();
    }

    if (send(sock, data.c_str(), data.length(), 0) < 0) {
        ROS_ERROR("Failed to send data to server");
        connected_ = false;  // Mark as disconnected
    } else {
        ROS_INFO("Data sent to server: %s", data.c_str());
    }
}
