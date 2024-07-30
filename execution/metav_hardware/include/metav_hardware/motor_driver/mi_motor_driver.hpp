#ifndef METAV_HARDWARE__MOTOR_DRIVER__MI_MOTOR_DRIVER_HPP_
#define METAV_HARDWARE__MOTOR_DRIVER__MI_MOTOR_DRIVER_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

#include <CanMessage.hpp>
#include <linux/can.h>

#include "angles/angles.h"

namespace metav_hardware {
class MiMotor {
  public:
    MiMotor(const std::string &motor_model, uint8_t mi_motor_id);
    MiMotor(const std::string &motor_model, uint8_t mi_motor_id, double Kp,
            double Kd);

    ~MiMotor() = default;

    std::string get_motor_model() const;
    uint8_t get_mi_motor_id() const;

    sockcanpp::CanMessage get_motor_enable_frame(uint8_t host_id) const;
    sockcanpp::CanMessage get_motor_disable_frame(uint8_t host_id) const;
    sockcanpp::CanMessage get_motor_command_frame(double position,
                                                  double velocity,
                                                  double effort) const;

    void set_motor_feedback(const sockcanpp::CanMessage &can_msg);

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

} // namespace metav_hardware

#endif // METAV_HARDWARE__MOTOR_DRIVER__MI_MOTOR_DRIVER_HPP_
