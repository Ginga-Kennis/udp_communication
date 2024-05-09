#include <ros/ros.h>
#include "base_udp.cpp" // BaseUdpクラスを含むヘッダファイルをインクルード
#include <string>

class UdpReceiver {
public:
    UdpReceiver(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);
    void timerCallback(const ros::TimerEvent& event);

private:
    BaseUdp baseUdp_; // BaseUdpのインスタンス
    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpReceiver::UdpReceiver(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port)
    : baseUdp_(local_ip, local_port, remote_ip, remote_port) // BaseUdpインスタンスを作成
{
    // タイマーをセットアップ（1秒ごとにコールバックを呼ぶ）
    timer_ = nh_.createTimer(ros::Duration(1.0), &UdpReceiver::timerCallback, this);
}

void UdpReceiver::timerCallback(const ros::TimerEvent& event) {
    // コールバックでメッセージを受信
    baseUdp_.udp_recv();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_receiver_node");

    // ノードパラメータ
    std::string local_ip = "192.168.2.100";
    int local_port = 8888;
    std::string remote_ip = "192.168.2.101";
    int remote_port = 8888;

    // UdpReceiverインスタンスを作成
    UdpReceiver udpReceiver(local_ip, local_port, remote_ip, remote_port);

    // メインループ
    ros::spin();
    return 0;
}
