#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <iostream>
#include <string>
#include <thread>
#include <netinet/in.h>
#include <unistd.h>

class TcpServer {
public:
    TcpServer(const std::string& address, int port);
    ~TcpServer();
    void start();
    void stop();

private:
    void acceptConnections();
    void handleClient(int client_socket);

    std::string address_;
    int port_;
    int server_socket_;
};

#endif // TCPSERVER_H
