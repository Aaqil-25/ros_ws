#ifndef DATASENDER_H
#define DATASENDER_H

#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ros/ros.h>

class DataSender {
public:
    DataSender(const std::string &server_ip, int server_port);
    ~DataSender();
    void sendData(const std::string &data);

private:
    void connectToServer();
    int sock;
    struct sockaddr_in serv_addr;
    std::string server_ip_;
    int server_port_;
    bool connected_;
};

#endif // DATASENDER_H
