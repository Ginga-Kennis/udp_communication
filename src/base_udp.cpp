#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <memory>

class BaseUdp {
public:
    static BaseUdp& getInstance(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);
    ~BaseUdp();

    void udp_send(const std::string& message);
    void udp_recv();
    void udp_bind();

private:
    BaseUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);
    BaseUdp(const BaseUdp&) = delete;
    BaseUdp& operator=(const BaseUdp&) = delete;

    int sock;
    struct sockaddr_in remote_addr;
    struct sockaddr_in local_addr;
    ros::NodeHandle nh_;
};

BaseUdp::BaseUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Set remote address
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port);
    remote_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());

    // Set local address
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_port);
    local_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());

    // Set socket to non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
}

BaseUdp& BaseUdp::getInstance(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port) {
    static BaseUdp instance(local_ip, local_port, remote_ip, remote_port);
    return instance;
}

BaseUdp::~BaseUdp() {
    close(sock);
}

void BaseUdp::udp_bind() {
    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sock);
        exit(EXIT_FAILURE);
    }
}

void BaseUdp::udp_send(const std::string& message) {
    int len = message.length();
    int n = sendto(sock, message.c_str(), len, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
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
