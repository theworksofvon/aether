#include <iostream>
#include <sys/socket.h> // Socket functions
#include <cstdlib> // for exit and EXIT_FAILURE
#include <unistd.h> //for read
#include <netinet/in.h> // for sockaddr_in

int main() {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cout << "Failed to create socket. err:" << errno << std::endl;
        exit(EXIT_FAILURE);
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(9999);

    if (bind(sockfd, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(sockaddr)) < 0) {
        std::cout << "Failed to bind to port 9999. errno:" << errno << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(sockfd, 10) < 0) {
        std::cout << "Failed to listen on socket. errono: " << errno << std::endl;
        exit(EXIT_FAILURE);
    }

    auto addrlen = sizeof(sockaddr);
    int connection = accept(sockfd, reinterpret_cast<struct sockaddr*>(&server_addr), reinterpret_cast<socklen_t*>(&addrlen));
    if (connection < 0) {
        std::cout << "Failed to grab connection. errno: " << errno << std::endl;
        exit(EXIT_FAILURE);
    }

    char buffer[100];
    auto bytesRead = read(connection, buffer, 100);
    std::cout << "The message was: " << buffer;

    std::string response = "good talking to you\n";

    send(connection, response.c_str(), response.size(), 0);


    close(connection);
    close(sockfd);

}


// tcp connection
// socket
// http server = text over tcp