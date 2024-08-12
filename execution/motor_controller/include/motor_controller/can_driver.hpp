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
#include <iostream>

/**
 * @brief A class for basic CAN bus manipulation.
 * This class is used to send CAN frames over the CAN bus.
 */
class CanDriver
{
private:
    int s;
    sockaddr_can addr;
    ifreq ifr;

public:
    /**
     * @brief Constructor for the CanDriver class.
     * Bind to the CAN cocket and cerr if it fails.
     * @param port The port number to use for the CAN driver. Default is 0.
     */
    CanDriver(int port)
    {
        s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // open the CAN socket

        std::string name = "can" + std::to_string(port); 
        std::strcpy(ifr.ifr_name, name.c_str());
        ioctl(s, SIOCGIFINDEX, &ifr); // set the interface name

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex; // set the socket address

        try {
            int bind_result = bind(s, (sockaddr*)&addr, sizeof(addr)); // bind the socket to the CAN interface
            if (bind_result == -1) {
                throw std::runtime_error("Error binding socket to CAN interface: " + std::string(strerror(errno)));
            }
        } catch (const std::exception& e) {
            std::cerr << "\033[1;31m" << e.what() << "\033[0m" << std::endl;
        }
    }

    /**
     * @brief Destructor for the CanDriver class.
     * This destructor closes the socket connection.
     */
    ~CanDriver()
    {
        close(s);
    }

    /**
     * @brief Sends a CAN frame over the CAN bus.
     * Cerr if it fails.
     * @param frame The CAN frame to be sent.
     */
    void send_frame(const can_frame &frame)
    {
        try {
            int write_result = write(s, &frame, sizeof(frame));
            if (write_result == -1) {
                throw std::runtime_error("Error sending to CAN: " + std::string(strerror(errno)));
            }
        } catch (const std::exception& e) {
            std::cerr << "\033[1;31m" << e.what() << "\033[0m" << std::endl;
        }
        write(s, &frame, sizeof(frame));
    }

    /**
     * @brief Receives a CAN frame from the CAN bus.
     * Cerr if it fails.
     * @param frame The CAN frame to be received.
     */
    void get_frame(can_frame &frame)
    {
        try {
            int read_result = read(s, &frame, sizeof(frame));
            if (read_result == -1) {
                throw std::runtime_error("Error receiving from CAN: " + std::string(strerror(errno)));
            }
        } catch (const std::exception& e) {
            std::cerr << "\033[1;31m" << e.what() << "\033[0m" << std::endl;
        }
    }
};

#endif // CAN_DRIVER_HPP