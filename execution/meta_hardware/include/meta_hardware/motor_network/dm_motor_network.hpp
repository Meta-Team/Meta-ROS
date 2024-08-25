#ifndef META_HARDWARE__MOTOR_NETWORK__DM_MOTOR_NETWORK_HPP_
#define META_HARDWARE__MOTOR_NETWORK__DM_MOTOR_NETWORK_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <sys/types.h>
#include <thread>
#include <unordered_map>
#include <vector>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/motor_driver/dm_motor_driver.hpp"

namespace meta_hardware {

class DmMotorNetwork {
  public:
    DmMotorNetwork(
        const std::string &can_network_name, uint8_t master_id,
        const std::vector<std::unordered_map<std::string, std::string>> &joint_params);
    ~DmMotorNetwork();

    /**
     * @brief Read the motor feedback
     * @param joint_id The joint ID of the motor
     * @return A tuple of (position, velocity, effort)
     */
    std::tuple<double, double, double> read(uint32_t joint_id) const;

    /**
     * @brief Write the motor command
     * @param joint_id The joint ID of the motor
     * @param position The position to write
     * @param velocity The velocity to write
     * @param effort The effort to write
     */
    void write_mit(uint32_t joint_id, double position, double velocity, double effort);
    void write_pos(uint32_t joint_id, double position);
    void write_vel(uint32_t joint_id, double velocity);

  private:
    // CAN driver
    std::unique_ptr<CanDriver> can_driver_;

    void rx_loop(std::stop_token stop_token);
    std::unique_ptr<std::jthread> rx_thread_;


    // [rx_can_id] -> dm_motor
    // This makes it easy to find the motor object in rx_loop
    std::map<uint32_t, std::shared_ptr<DmMotor>> motor_id2motor_;

    // [joint_id] -> dm_motor
    // This makes it easy to find the motor object in read() and write()
    std::vector<std::shared_ptr<DmMotor>> dm_motors_;

    // Master ID
    uint32_t master_id_;
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_NETWORK__DM_MOTOR_NETWORK_HPP_
