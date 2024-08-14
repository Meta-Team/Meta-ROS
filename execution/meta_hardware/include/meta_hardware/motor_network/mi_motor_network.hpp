#ifndef META_HARDWARE__MOTOR_NETWORK__MI_MOTOR_NETWORK_HPP_
#define META_HARDWARE__MOTOR_NETWORK__MI_MOTOR_NETWORK_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/motor_driver/mi_motor_driver.hpp"

namespace meta_hardware {

class MiMotorNetwork {
  public:
    MiMotorNetwork(
        const std::string &can_network_name, uint8_t host_id,
        const std::vector<std::unordered_map<std::string, std::string>> &joint_params);
    ~MiMotorNetwork();

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
    void write_dyn(uint32_t joint_id, double position, double velocity, double effort);
    void write_pos(uint32_t joint_id, double position);
    void write_vel(uint32_t joint_id, double velocity);

  private:
    // CAN driver
    std::unique_ptr<CanDriver> can_driver_;

    void rx_loop(std::stop_token stop_token);
    std::unique_ptr<std::jthread> rx_thread_;

    using mi_can_frame = MiMotor::mi_can_frame;
    void process_mi_frame(const mi_can_frame &can_msg);
    void process_mi_info_frame(const mi_can_frame &can_msg);
    void process_mi_fb_frame(const mi_can_frame &can_msg);

    // [motor_id] -> mi_motor
    // This makes it easy to find the motor object in rx_loop
    std::map<uint8_t, std::shared_ptr<MiMotor>> motor_id2motor_;

    // [joint_id] -> mi_motor
    // This makes it easy to find the motor object in read() and write()
    std::vector<std::shared_ptr<MiMotor>> mi_motors_;

    // Host ID
    uint8_t host_id_;
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_NETWORK__MI_MOTOR_NETWORK_HPP_
