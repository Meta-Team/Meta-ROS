#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/can_driver/can_exceptions.hpp"
#include "meta_hardware/motor_driver/brt_encoder_driver.hpp"
#include "meta_hardware/motor_network/brt_encoder_network.hpp"

namespace meta_hardware {
using std::string;
using std::unordered_map;
using std::vector;

BrtEncoderNetwork::BrtEncoderNetwork(const string &can_network_name,
                               const vector<unordered_map<string, string>> &joint_params){

    std::vector<can_filter> filters;

    // Initialize BRITTER encoder drivers
    for (const auto &joint_param : joint_params) {
        uint32_t brt_encoder_id = std::stoi(joint_param.at("encoder_id"));

        auto brt_encoder = std::make_shared<BrtEncoder>(joint_param);
        encoder_id2encoder_[brt_encoder_id] = brt_encoder;
        brt_encoders_.emplace_back(brt_encoder);

    }

    filters.push_back({.can_id = 0x001, .can_mask = CAN_EFF_MASK});

    // Initialize CAN driver
    can_driver_ = std::make_unique<CanDriver>(can_network_name, false, filters);

    // Start RX thread
    rx_thread_ =
        std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

BrtEncoderNetwork::~BrtEncoderNetwork() = default;

double BrtEncoderNetwork::read(size_t joint_id) const {
    return brt_encoders_[joint_id]->get_encoder_feedback();
}

void BrtEncoderNetwork::rx_loop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        try {
            can_frame can_msg = can_driver_->read(2000);

            uint8_t encoder_id = can_msg.data[1];
            encoder_id2encoder_.at(encoder_id)->set_encoder_feedback(can_msg);
        } catch (CanIOTimedOutException & /*e*/) {
            std::cerr << "Timed out waiting for BRITTER encoder feedback." << std::endl;
        } catch (CanIOException &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

} // namespace meta_hardware
