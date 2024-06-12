#include "rosbag_reader/TcpServer.h"
#include <ros/ros.h>
#include <arpa/inet.h>  // For inet_addr
#include <cstring>      // For memset
#include <unistd.h>     // For close
#include <thread>       // For std::thread

TcpServer::TcpServer(const std::string &address, int port)
    : address_(address), port_(port), server_socket_(-1) {}

TcpServer::~TcpServer() {
    stop();
}

void TcpServer::start() {
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ == -1) {
        ROS_ERROR("Could not create socket: %s", strerror(errno));
        return;
    }

    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    server_addr.sin_addr.s_addr = inet_addr(address_.c_str());

    if (bind(server_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Bind failed: %s", strerror(errno));
        return;
    }

    if (listen(server_socket_, 3) < 0) {
        ROS_ERROR("Listen failed: %s", strerror(errno));
        return;
    }

    ROS_INFO("Server listening on %s:%d", address_.c_str(), port_);

    // Use a separate thread to handle incoming connections
    std::thread accept_thread(&TcpServer::acceptConnections, this);
    accept_thread.detach();
}

void TcpServer::acceptConnections() {
    sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    while (true) {
        int client_socket = accept(server_socket_, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_socket < 0) {
            ROS_ERROR("Accept failed: %s", strerror(errno));
            continue;
        }
        ROS_INFO("Accepted new connection");

        // Handle the client in a new thread
        std::thread client_thread(&TcpServer::handleClient, this, client_socket);
        client_thread.detach();
    }
}

void TcpServer::handleClient(int client_socket) {
    char buffer[1024];
    int read_size;

    while ((read_size = recv(client_socket, buffer, sizeof(buffer), 0)) > 0) {
        buffer[read_size] = '\0';
        ROS_INFO("Received: %s", buffer);
    }

    if (read_size == 0) {
        ROS_INFO("Client disconnected");
    } else if (read_size < 0) {
        ROS_ERROR("Recv failed: %s", strerror(errno));
    }

    close(client_socket);
}

void TcpServer::stop() {
    if (server_socket_ != -1) {
        close(server_socket_);
    }
}
