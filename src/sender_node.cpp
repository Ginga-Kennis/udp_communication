#include <ros/ros.h>
#include "base_udp.cpp" // BaseUdpクラスを含むヘッダファイルをインクルード
#include <string>

class UdpSender {
public:
    UdpSender(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);
    void timerCallback(const ros::TimerEvent& event);

private:
    BaseUdp baseUdp_; // BaseUdpのインスタンス
    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpSender::UdpSender(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port)
    : baseUdp_(local_ip, local_port, remote_ip, remote_port) // BaseUdpインスタンスを作成
{
    // タイマーをセットアップ（1秒ごとにコールバックを呼ぶ）
    timer_ = nh_.createTimer(ros::Duration(1.0), &UdpSender::timerCallback, this);
}

void UdpSender::timerCallback(const ros::TimerEvent& event) {
    // コールバックでメッセージを送信
    baseUdp_.udp_send("Hello, this is a timed message!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_sender_node");

    // ノードパラメータ
    std::string local_ip = "192.168.2.100";
    int local_port = 8888;
    std::string remote_ip = "192.168.2.101";
    int remote_port = 8888;

    // UdpSenderインスタンスを作成
    UdpSender udpSender(local_ip, local_port, remote_ip, remote_port);

    // メインループ
    ros::spin();
    return 0;
}
