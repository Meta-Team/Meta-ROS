#include "../include/dm_motor_controller/can_driver.h"

CanDriver::CanDriver()
{
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // open the CAN socke

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr); // set the interface name

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex; // set the socket address

    bind(s, (struct sockaddr *)&addr, sizeof(addr)); // bind the socket to the CAN interface
}

CanDriver::~CanDriver()
{
    // close the CAN socket
    close(s);
}

void CanDriver::send_frame(can_frame frame)
{
    write(s, &frame, sizeof(frame));
}