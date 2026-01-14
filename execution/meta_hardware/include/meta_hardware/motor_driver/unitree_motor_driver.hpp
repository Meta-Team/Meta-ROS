#ifndef META_HARDWARE__MOTOR_DRIVER__UNITREE_MOTOR_DRIVER_HPP_
#define META_HARDWARE__MOTOR_DRIVER__UNITREE_MOTOR_DRIVER_HPP_

#include <string>
#include <unordered_map>
#include <tuple>

#include "unitree_sdk/unitreeMotor.h"

namespace meta_hardware {

class UnitreeMotor {
  public:
    explicit UnitreeMotor(const std::unordered_map<std::string, std::string> &motor_param);
    ~UnitreeMotor() = default;

    /**
     * @brief Initialize a MotorCmd struct with defaults (ID, Mode, Type)
     */
    void initialize_cmd(MotorCmd& cmd, MotorData& data);

    /**
     * @brief Update the MotorCmd with control values
     */
    void set_motor_command(MotorCmd& cmd, double position, double velocity, double effort);

    /**
     * @brief Parse MotorData into standard state
     * @return {position, velocity, effort} (After removing internal motor gear ratio)
     */
    std::tuple<double, double, double> get_motor_feedback(const MotorData& data) const;

  private:
    uint8_t motor_id_{0};
    double kp_{0.0};
    double kd_{0.0};
    double gear_ratio_{1.0};
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_DRIVER__UNITREE_MOTOR_DRIVER_HPP_