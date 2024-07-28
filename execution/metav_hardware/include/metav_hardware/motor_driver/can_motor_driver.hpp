#ifndef METAV_HARDWARE__MOTOR_DRIVER__CAN_MOTOR_DRIVER_HPP_
#define METAV_HARDWARE__MOTOR_DRIVER__CAN_MOTOR_DRIVER_HPP_

namespace metav_hardware {
/**
 * @brief Interface for a CAN motor driver
 * This class basically does nothing other than grouping all the CAN motors
 * together. The actual motor driver is implemented in the derived class.
 * This class does not provide any polymorphic behavior. Since all the motor
 * drivers directly interact with their respective CanMotorNetwork, no
 * polymorphic behavior is required.
 */
class CanMotorDriver {
  public:
    virtual ~CanMotorDriver() = default;
};
} // namespace metav_hardware

#endif // METAV_HARDWARE__MOTOR_DRIVER__CAN_MOTOR_DRIVER_HPP_
