#ifndef META_HARDWARE__MOTOR_DRIVER__MI_MOTOR_DRIVER_HPP_
#define META_HARDWARE__MOTOR_DRIVER__MI_MOTOR_DRIVER_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>

#include <linux/can.h>

#include "angles/angles.h"

namespace meta_hardware {
class MiMotor {
  public:
    explicit MiMotor(const std::unordered_map<std::string, std::string> &motor_param);

    ~MiMotor() = default;

    can_frame get_motor_enable_frame(uint8_t host_id) const;
    can_frame get_motor_disable_frame(uint8_t host_id) const;
    can_frame get_motor_dyn_frame(double position, double velocity, double effort) const;
    can_frame get_motor_pos_frame(double position) const;
    can_frame get_motor_vel_frame(double velocity) const;

    void set_motor_feedback(const can_frame &can_msg);

    std::tuple<double, double, double> get_motor_feedback() const;

  private:
    // Motor information
    std::string motor_model_;
    uint32_t mi_motor_id_;

    // Motor feedback
    double position_{0.0};
    double velocity_{0.0};
    double torque_{0.0};

    // Motor control parameters
    uint16_t Kp_raw_{0};
    uint16_t Kd_raw_{0};
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_DRIVER__MI_MOTOR_DRIVER_HPP_
