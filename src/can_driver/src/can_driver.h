#include <can_interfaces/msg/detail/mit_data__struct.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <rclcpp/parameter.hpp>
#include <rclcpp/subscription.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "can_data.h"

class CanDriver : public rclcpp::Node
{
public:
    enum mode
    {
        MIT,
        POSVEL,
        VEL
    };

    CanDriver();

    ~CanDriver();

    void send_frame();

    // void set_frame_data(mode motor_mode);

private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_frame frame_read;

    rclcpp::Subscription<can_interfaces::msg::MITData>::SharedPtr subscription_mit_;
    rclcpp::Subscription<can_interfaces::msg::PosVelData>::SharedPtr subscription_posvel_;
    rclcpp::Subscription<can_interfaces::msg::VelData>::SharedPtr subscription_vel_;

    rclcpp::Publisher<can_interfaces::msg::Feedback>::SharedPtr publisher_feedback_;
};