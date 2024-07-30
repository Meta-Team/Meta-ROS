#ifndef METAV_HARDWARE__MOTOR_NETWORK__CAN_MOTOR_NETWORK_HPP_
#define METAV_HARDWARE__MOTOR_NETWORK__CAN_MOTOR_NETWORK_HPP_

#include <string>
#include <unordered_map>

namespace metav_hardware {

/**
 * @brief The abstract class for the CAN motor network. A CAN motor network
 * means a series of motors from the same vendor on the same CAN bus. Unifying
 * their communication is beneficial and sometimes even necessary (especially
 * for DJI motors).
 */
class CanMotorNetwork {
  public:
    virtual ~CanMotorNetwork() = default;

    /**
     * @brief Add a motor to the motor network, see detailed description in the
     * derived class.
     * @param joint_id The joint ID of the motor
     * @param motor_params The parameters of the motor
     */
    virtual void
    add_motor(uint32_t joint_id,
              const std::unordered_map<std::string, std::string> &motor_params);

    /**
     * @brief Read the motor feedback
     * @param joint_id The joint ID of the motor
     * @return A tuple of (position, velocity, effort)
     */
    virtual std::tuple<double, double, double>
    read(uint32_t joint_id) const = 0;

    /**
     * @brief Write the motor command
     * @param joint_id The joint ID of the motor
     * @param effort The effort to write
     */
    virtual void write(uint32_t joint_id, double position, double velocity,
                       double effort) = 0;

    /**
     * @brief Transmit the motor commands (optional, useful for DJI motors).
     */
    virtual void tx() = 0;
};
} // namespace metav_hardware

#endif // METAV_HARDWARE__MOTOR_NETWORK__CAN_MOTOR_NETWORK_HPP_
