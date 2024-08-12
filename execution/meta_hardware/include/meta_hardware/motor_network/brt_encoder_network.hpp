#ifndef META_HARDWARE__MOTOR_NETWORK__BRT_ENCODER_NETWORK_HPP_
#define META_HARDWARE__MOTOR_NETWORK__BRT_ENCODER_NETWORK_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/motor_driver/brt_encoder_driver.hpp"

namespace meta_hardware {

class BrtEncoderNetwork {
  public:
    BrtEncoderNetwork(
        const std::string &can_network_name,
        const std::vector<std::unordered_map<std::string, std::string>> &joint_params);
    ~BrtEncoderNetwork();

    /**
     * @brief Read the motor feedback
     * @param joint_id The joint ID of the motor
     * @return A tuple of (position, velocity, effort)
     */
    double read(size_t joint_id) const;

  private:
    // CAN driver
    std::unique_ptr<CanDriver> can_driver_;

    void rx_loop(std::stop_token stop_token);
    std::unique_ptr<std::jthread> rx_thread_;

    // [motor_id] -> mi_motor
    // This makes it easy to find the motor object in rx_loop
    std::map<uint32_t, std::shared_ptr<BrtEncoder>> encoder_id2encoder_;

    // [joint_id] -> mi_motor
    // This makes it easy to find the motor object in read() and write()
    std::vector<std::shared_ptr<BrtEncoder>> brt_encoders_;

    // Host ID
    uint32_t host_id_;
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_NETWORK__BRT_ENCODER_NETWORK_HPP_
