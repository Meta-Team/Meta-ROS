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
        std::string can_interface,
        uint32_t master_id,
        const std::vector<std::unordered_map<std::string, std::string>>
            &motor_params);
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
     * @param effort The effort to write
     */
    void write(uint32_t joint_id, double effort);

    /**
     * @brief Transmit the motor commands
     */
    void tx();

  private:
    void rx_loop();
    std::unique_ptr<std::jthread> rx_thread_;
    std::atomic<bool> rx_thread_running_{true};

    uint32_t master_id_;

    // Five CAN frames for tx
    // 0x1FE: GM6020(current) motor 1-4
    // 0x1FF: M3508/M2006 motor 5-8 / GM6020(voltage) motor 1-4
    // 0x200: M3508/M2006 motor 1-4
    // 0x2FE: GM6020(current) motor  5-8
    // 0x2FF: GM6020(voltage) motor 5-8
    std::unordered_map<uint32_t, can_frame> tx_frames_;

    // CAN driver
    std::unique_ptr<CanDriver> can_driver_;

    // [rx_can_id] -> dm_motor
    // This makes it easy to find the motor object in rx_loop
    std::map<uint32_t, std::shared_ptr<DmMotor>> motor_id2motor_;

    // [joint_id] -> dm_motor
    // This makes it easy to find the motor object in read() and write()
    std::vector<std::shared_ptr<DmMotor>> motors_;
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_NETWORK__DM_MOTOR_NETWORK_HPP_
