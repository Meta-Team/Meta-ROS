#ifndef CAN_DRIVER_HPP
#define CAN_DRIVER_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string>
#include <cstring>
#include <unistd.h>

class CanDriver
{
private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

public:
    CanDriver(int port = 0)
    {
        s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // open the CAN socket

        std::string name = "can" + std::to_string(port); 
        std::strcpy(ifr.ifr_name, name.c_str());
        ioctl(s, SIOCGIFINDEX, &ifr); // set the interface name

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex; // set the socket address

        int bind_result = bind(s, (struct sockaddr *)&addr, sizeof(addr)); // bind the socket to the CAN interface
        if (bind_result == -1) perror("Error binding socket to CAN interface");
    }

    ~CanDriver()
    {
        // close the CAN socket
        close(s);
    }

    void send_frame(can_frame frame)
    {
        write(s, &frame, sizeof(frame));
    }
};

#endif // CAN_DRIVER_HPP