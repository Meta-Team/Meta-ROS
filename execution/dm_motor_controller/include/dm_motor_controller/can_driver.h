#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <linux/can.h>
#include <linux/can/raw.h>
// #include <rclcpp/parameter.hpp>
// #include <rclcpp/subscription.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

class CanDriver
{
private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

public:
    CanDriver();

    ~CanDriver();

    void send_frame(can_frame frame);    
};

#endif // CAN_DRIVER_H