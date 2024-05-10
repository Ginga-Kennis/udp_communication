#include <ros/ros.h>
#include "base_udp.cpp" // Include the BaseUdp class
#include <string>

class UdpSender {
public:
    UdpSender(const std::string remote_ip, const int remote_port);
    void timerCallback(const ros::TimerEvent& event);

private:
    BaseUdp baseUdp_; // BaseUdp instance
    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpSender::UdpSender(const std::string remote_ip, const int remote_port)
    : baseUdp_(remote_ip, remote_port) { // Initializing the instance in constructor
    // No binding needed for sender

    // Setup the timer (calls the callback every second)
    timer_ = nh_.createTimer(ros::Duration(0.01), &UdpSender::timerCallback, this);
}

void UdpSender::timerCallback(const ros::TimerEvent& event) {
    // Send message in callback
    baseUdp_.udp_send("Hello from laptop!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_sender_node");

    // Node parameters

    std::string remote_ip = "192.168.2.101";
    int remote_port = 8888;

    // Create UdpSender instance
    UdpSender udpSender(remote_ip, remote_port);

    // Main loop
    ros::spin();
    return 0;
}
