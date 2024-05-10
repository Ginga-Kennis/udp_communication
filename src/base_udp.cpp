#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>

class BaseUdp {
public:
    BaseUdp(const std::string ip_address, const int port);
    ~BaseUdp();

    void udp_send(const std::string& message);
    void udp_recv();
    void udp_bind();

private:
    int sock;
    struct sockaddr_in addr;
};

BaseUdp::BaseUdp(const std::string ip_address, const int port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Set remote address
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip_address.c_str());

    // Set socket to non-blocking mode
    // int flags = fcntl(sock, F_GETFL, 0);
    // fcntl(sock, F_SETFL, flags | O_NONBLOCK);
}

BaseUdp::~BaseUdp() {
    close(sock);
}

void BaseUdp::udp_bind() {
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sock);
        exit(EXIT_FAILURE);
    }
}

void BaseUdp::udp_send(const std::string& message) {
    int len = message.length();
    int n = sendto(sock, message.c_str(), len, 0, (struct sockaddr *)&addr, sizeof(addr));
    if (n < 0) {
        std::cerr << "Failed to send packet" << std::endl;
    }
}

void BaseUdp::udp_recv() {
    char buffer[1024];
    struct sockaddr_in src_addr;
    socklen_t src_addr_len = sizeof(src_addr);

    int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&src_addr, &src_addr_len);
    if (len > 0) {
        buffer[len] = '\0';
        std::cout << "Received message: " << buffer << std::endl;
    } else if (len < 0 && errno != EWOULDBLOCK) {
        std::cerr << "Failed to receive message" << std::endl;
    }
}
