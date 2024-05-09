#include <ros/ros.h>
#include "base_udp.cpp" // Include the BaseUdp class
#include <string>

class UdpSender {
public:
    UdpSender(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);
    void timerCallback(const ros::TimerEvent& event);

private:
    BaseUdp* baseUdp_; // Pointer to the singleton instance of BaseUdp
    ros::NodeHandle nh_;
    ros::Timer timer_;
};

UdpSender::UdpSender(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port) {
    // Get singleton instance (no binding needed for sender)
    baseUdp_ = &BaseUdp::getInstance(local_ip, local_port, remote_ip, remote_port);

    // Setup the timer (calls the callback every second)
    timer_ = nh_.createTimer(ros::Duration(1.0), &UdpSender::timerCallback, this);
}

void UdpSender::timerCallback(const ros::TimerEvent& event) {
    // Send message in callback
    baseUdp_->udp_send("Hello from laptop!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_sender_node");

    // Node parameters
    std::string local_ip = "192.168.2.100";
    int local_port = 8888;
    std::string remote_ip = "192.168.2.101";
    int remote_port = 8888;

    // Create UdpSender instance
    UdpSender udpSender(local_ip, local_port, remote_ip, remote_port);

    // Main loop
    ros::spin();
    return 0;
}
