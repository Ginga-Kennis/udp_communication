#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <memory>
#include <mutex>

class BaseUdp {
public:
    // 静的なメソッドでインスタンスを取得する
    static BaseUdp& getInstance(const std::string& local_ip = "127.0.0.1", int local_port = 0, 
                                const std::string& remote_ip = "127.0.0.1", int remote_port = 0);
    ~BaseUdp();

    void udp_send(const std::string& message);
    void udp_recv();

private:
    // プライベートコンストラクタ：外部からの直接のインスタンス化を防ぐ
    BaseUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);

    // コピーコンストラクタと代入演算子を削除：複製を防ぐ
    BaseUdp(const BaseUdp&) = delete;
    BaseUdp& operator=(const BaseUdp&) = delete;

    int sock;
    struct sockaddr_in remote_addr;
    struct sockaddr_in local_addr;
    ros::NodeHandle nh_;

    // 静的なインスタンスポインタ
    static std::unique_ptr<BaseUdp> instance;
    // 静的なミューテックス
    static std::mutex instance_mutex;
};

// 静的メンバ変数の定義と初期化
std::unique_ptr<BaseUdp> BaseUdp::instance = nullptr;
std::mutex BaseUdp::instance_mutex;

BaseUdp::BaseUdp(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    // リモートアドレスの設定
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port);
    remote_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());

    // ローカルアドレスの設定
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_port);
    local_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sock);
        exit(EXIT_FAILURE);
    }

    // ソケットをノンブロッキングモードに設定
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
}

BaseUdp::~BaseUdp() {
    close(sock);
}

BaseUdp& BaseUdp::getInstance(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port) {
    // インスタンス生成時の排他制御
    std::lock_guard<std::mutex> lock(instance_mutex);
    // インスタンスがまだ存在しない場合にのみ生成
    if (instance == nullptr) {
        instance.reset(new BaseUdp(local_ip, local_port, remote_ip, remote_port));
    }
    // 既存のインスタンスを返す
    return *instance;
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

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "base_udp_node");

//     std::string local_ip = "192.168.2.100";
//     int local_port = 8888;

//     std::string remote_ip = "192.168.2.101";
//     int remote_port = 8888;

//     // シングルトンインスタンスを取得
//     BaseUdp& baseUdp = BaseUdp::getInstance(local_ip, local_port, remote_ip, remote_port);

//     baseUdp.udp_send("Hello, this is a test message!");

//     ros::Duration(1.0).sleep();
//     baseUdp.udp_recv();

//     ros::spin();
//     return 0;
// }
