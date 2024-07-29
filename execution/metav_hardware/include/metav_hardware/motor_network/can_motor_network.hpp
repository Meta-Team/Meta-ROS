#ifndef METAV_HARDWARE__MOTOR_NETWORK__CAN_MOTOR_NETWORK_HPP_
#define METAV_HARDWARE__MOTOR_NETWORK__CAN_MOTOR_NETWORK_HPP_

#include <string>

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
     * @param motor_model The model name of the motor
     * @param motor_id The internal motor ID of the motor
     * @param joint_id The joint ID of the motor
     */
    virtual void add_motor(std::string motor_model, uint32_t motor_id,
                           uint32_t joint_id) = 0;

    /**
     * @brief Initialize the RX and TX threads
     */
    virtual void init_rx() = 0;

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
    virtual void write(uint32_t joint_id, double effort) = 0;

    /**
     * @brief Transmit the motor commands.
     */
    virtual void tx() = 0;
};
} // namespace metav_hardware

#endif // METAV_HARDWARE__MOTOR_NETWORK__CAN_MOTOR_NETWORK_HPP_
