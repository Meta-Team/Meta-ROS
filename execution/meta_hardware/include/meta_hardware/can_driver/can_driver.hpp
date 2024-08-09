#ifndef META_HARDWARE_CAN_DRIVER_CAN_DRIVER_HPP
#define META_HARDWARE_CAN_DRIVER_CAN_DRIVER_HPP
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace meta_hardware {
class CanDriver {
  public:
    CanDriver(std::string can_interface, bool join_filters = false,
              const std::vector<can_filter> &can_filters = {}) {
        // Create a socket
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (can_socket_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        // Get the interface index
        ifreq ifr;
        std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ);

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            throw std::runtime_error("Failed to get interface index");
        }

        // Bind the socket to the interface
        sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, reinterpret_cast<sockaddr *>(&addr),
                 sizeof(addr)) < 0) {
            throw std::runtime_error("Failed to bind socket");
        }

        // Set the socket to non-blocking
        int flags = fcntl(can_socket_, F_GETFL, 0);
        if (flags < 0) {
            throw std::runtime_error("Failed to get socket flags");
        }

        if (fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
            throw std::runtime_error("Failed to set socket flags");
        }

        // Set the filters
        if (join_filters && !can_filters.empty()) {
            if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
                           &can_filters[0],
                           can_filters.size() * sizeof(can_filter)) < 0) {
                throw std::runtime_error("Failed to set filters");
            }
        } else if (!can_filters.empty()) {
            if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER,
                           &can_filters[0],
                           can_filters.size() * sizeof(can_filter)) < 0) {
                throw std::runtime_error("Failed to set filters");
            }
        }

        // Set up the pollfd struct
        poll_fd_.fd = can_socket_;
        poll_fd_.events = POLLIN;
    }

    can_frame read() {
        can_frame frame;
        while (true) {
            // Wait indefinitely until a frame is available
            poll(&poll_fd_, 1, -1);

            if (poll_fd_.revents & POLLIN) {
                ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));

                // Sometimes read can return EAGAIN or EWOULDBLOCK even though
                // poll says there is data available.
                // And read may also fail for other reasons.
                if (nbytes < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        continue;
                    } else {
                        throw std::runtime_error("Failed to read frame");
                    }
                }

                break; // If everything went well, break out of the loop
            }
        }
        return frame;
    }

    ssize_t write(const can_frame &frame) {
        // Write is completely non-blocking.
        // If any error occurs, the caller should handle it via exception.
        ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
        if (nbytes < 0) {
            throw std::runtime_error("Failed to write frame");
        }
        return nbytes;
    }

    ~CanDriver() {
        if (can_socket_ >= 0) {
            close(can_socket_);
        }
    }

  private:
    int can_socket_;
    pollfd poll_fd_;
};
} // namespace meta_hardware

#endif // META_HARDWARE_CAN_DRIVER_CAN_DRIVER_HPP