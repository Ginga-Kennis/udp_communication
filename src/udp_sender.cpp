#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class UdpSender {
public:
    UdpSender(const std::string& ip, int port);
    ~UdpSender();

    void timerCallback(const ros::TimerEvent& event);

private:
    int sock;
    struct sockaddr_in addr;
    int n;

    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpSender::UdpSender(const std::string& ip, int port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cout << "Failed to create socket" << std::endl;
        exit(EXIT_FAILURE);  // ソケット作成に失敗した場合は終了
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip.c_str());

    timer_ = nh_.createTimer(ros::Duration(0.01), &UdpSender::timerCallback, this);
}

UdpSender::~UdpSender() {
    close(sock);
}

void UdpSender::timerCallback(const ros::TimerEvent& event){
    const char* message = "HELLO";
    int len = strlen(message);
    int n = sendto(sock, message, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    if (n < 0) {
        std::cout << "Failed to send packet" << std::endl;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_sender_node");

    std::string ip = "127.0.0.1";
    int port = 4001;

    UdpSender sender(ip, port);

    ros::spin();
    return 0;
}
