#include <ros/ros.h>
#include "base_udp.cpp" // BaseUdpクラスを含むヘッダファイルをインクルード
#include <string>

class UdpReceiver {
public:
    UdpReceiver();
    void timerCallback(const ros::TimerEvent& event);

private:
    BaseUdp& baseUdp_; // BaseUdpのシングルトンインスタンスへの参照
    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpReceiver::UdpReceiver()
    : baseUdp_(BaseUdp::getInstance()) // BaseUdpインスタンスを取得
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

    // UdpReceiverインスタンスを作成
    UdpReceiver udpReceiver;

    // メインループ
    ros::spin();
    return 0;
}
