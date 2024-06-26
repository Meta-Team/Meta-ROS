#ifndef CAN_PORT_H
#define CAN_PORT_H

#include "dji_controller/can_driver.hpp"
#include <memory>

class CanPort
{
private:
    std::unique_ptr<CanDriver> can_driver_; /**< Pointer to the CAN driver instance. */

    /**
     * @brief Initialize a CAN frame.
     * @param frame_id The ID of the frame.
     * @return A pointer to the CAN frame.
     */
    static can_frame init_frame(int frame_id)
    {
        can_frame frame;
        frame.can_id = frame_id;
        frame.can_dlc = 8;
        for (auto& data : frame.data) data = 0;
        return frame;
    }

public:
    can_frame tx_frame_200, tx_frame_1ff, tx_frame_2ff; /**< CAN frames for transmitting data. */
    can_frame rx_frame; /**< CAN frame for receiving data. */

    /**
     * @brief Constructor for the CanPort class.
     * Initialize the CAN driver and CAN frames.
     * @param port The port number to use for the CAN driver.
     */
    CanPort(int port)
    {
        can_driver_ = std::make_unique<CanDriver>(port);
        tx_frame_200 = init_frame(0x200);
        tx_frame_1ff = init_frame(0x1ff);
        tx_frame_2ff = init_frame(0x2ff);
        rx_frame = init_frame(0);
    }

    /**
     * @brief Transmit data to the CAN bus.
     */
    void tx()
    {
        can_driver_->send_frame(tx_frame_200);
        can_driver_->send_frame(tx_frame_1ff);
        can_driver_->send_frame(tx_frame_2ff);
    }

    /**
     * @brief Receive data from the CAN bus.
     */
    void rx()
    {
        can_driver_->get_frame(rx_frame);
    }
};

#endif // CAN_PORT_H