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

#include "meta_hardware/can_driver/can_exceptions.hpp"

namespace meta_hardware {
class CanDriver {
  public:
    CanDriver(std::string can_interface, bool join_filters = false,
              const std::vector<can_filter> &can_filters = {}) {
        // Create a socket
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (can_socket_ < 0) {
            throw CanException("Failed to create socket: " + std::string(strerror(errno)));
        }

        // Get the interface index
        ifreq ifr;
        std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ);

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            throw CanException("Failed to get interface index: " + std::string(strerror(errno)));
        }

        // Bind the socket to the interface
        sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            throw CanException("Failed to bind socket");
        }

        // Set the socket to non-blocking
        int flags = fcntl(can_socket_, F_GETFL, 0);
        if (flags < 0) {
            throw CanException("Failed to get socket flags: " + std::string(strerror(errno)));
        }

        if (fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
            throw CanException("Failed to set socket flags: " + std::string(strerror(errno)));
        }

        // Set the filters
        if (join_filters && !can_filters.empty()) {
            if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
                           &can_filters[0],
                           can_filters.size() * sizeof(can_filter)) < 0) {
                throw CanException("Failed to set filters: " + std::string(strerror(errno)));
            }
        } else if (!can_filters.empty()) {
            if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &can_filters[0],
                           can_filters.size() * sizeof(can_filter)) < 0) {
                throw CanException("Failed to set filters: " + std::string(strerror(errno)));
            }
        }

        // Set up the pollfd struct
        poll_fd_.fd = can_socket_;
        poll_fd_.events = POLLIN;
    }

    can_frame read(int timeout_ms) {
        can_frame frame;
        while (true) {
            // Wait until a frame is available
            int poll_ret = poll(&poll_fd_, 1, timeout_ms);

            if (poll_ret == 0) {
                throw CanIOTimedOutException("Timed out waiting for a frame");
            } else if (poll_ret < 0) {
                throw CanIOException("Failed to poll for frame: " + std::string(strerror(errno)));
            }

            if (poll_fd_.revents & POLLIN) {
                ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));

                // Sometimes read can return EAGAIN or EWOULDBLOCK even though
                // poll says there is data available.
                // And read may also fail for other reasons.
                if (nbytes < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        continue;
                    } else {
                        throw CanIOException("Failed to read frame: " + std::string(strerror(errno)));
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
            throw CanIOException("Failed to write frame: " + std::string(strerror(errno)));
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