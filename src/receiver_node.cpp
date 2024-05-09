#include <ros/ros.h>
#include "base_udp.cpp" // BaseUdp class
#include <string>

class UdpReceiver {
public:
    UdpReceiver(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);
    void timerCallback(const ros::TimerEvent& event);

private:
    BaseUdp* baseUdp_; // Pointer to the singleton instance of BaseUdp
    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpReceiver::UdpReceiver(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port) {
    // Get singleton instance and bind the UDP socket
    baseUdp_ = &BaseUdp::getInstance(local_ip, local_port, remote_ip, remote_port);
    baseUdp_->udp_bind();

    // Setup the timer (calls the callback every second)
    timer_ = nh_.createTimer(ros::Duration(1.0), &UdpReceiver::timerCallback, this);
}

void UdpReceiver::timerCallback(const ros::TimerEvent& event) {
    // Receive message in callback
    baseUdp_->udp_recv();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_receiver_node");

    // Node parameters
    std::string local_ip = "192.168.2.100";
    int local_port = 8888;
    std::string remote_ip = "192.168.2.101";
    int remote_port = 8888;

    // Create UdpReceiver instance
    UdpReceiver udpReceiver(local_ip, local_port, remote_ip, remote_port);

    // Main loop
    ros::spin();
    return 0;
}
