#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>

class UdpReceiver {
public:
    UdpReceiver(int port);
    ~UdpReceiver();

    void timerCallback(const ros::TimerEvent& event);

private:
    int sock;
    struct sockaddr_in local_addr;
    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpReceiver::UdpReceiver(int port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(port);
    local_addr.sin_addr.s_addr = INADDR_ANY; // 任意のアドレスからのデータを受け付ける

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sock);
        exit(EXIT_FAILURE);
    }

    // ソケットを非ブロッキングモードに設定
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // タイマーの設定 (0.01秒ごと)
    timer_ = nh_.createTimer(ros::Duration(0.01), &UdpReceiver::timerCallback, this);
}

UdpReceiver::~UdpReceiver() {
    close(sock);
}

void UdpReceiver::timerCallback(const ros::TimerEvent& event) {
    char buffer[1024];
    struct sockaddr_in src_addr;
    socklen_t src_addr_len = sizeof(src_addr);

    int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&src_addr, &src_addr_len);
    if (len > 0) {
        buffer[len] = '\0';  // Ensure null-terminated string
        std::cout << "Received message: " << buffer << std::endl;
    } else if (len < 0 && errno != EWOULDBLOCK) {
        std::cerr << "Failed to receive message" << std::endl;
    }
    // EWOULDBLOCKエラーは無視する（データがない正常な状態）
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_receiver_node");

    int port = 4001; // 受信するポート番号
    UdpReceiver receiver(port);

    ros::spin();
    return 0;
}
