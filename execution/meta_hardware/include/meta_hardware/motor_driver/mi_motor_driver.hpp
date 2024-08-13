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
    struct [[gnu::packed]] mi_can_frame {
        struct [[gnu::packed]] mi_can_id {
            uint8_t id : 8;
            uint16_t data : 16;
            uint8_t mode : 5;
            uint8_t err : 1 = 0;
            uint8_t rtr : 1 = 0;
            uint8_t eff : 1 = 1;
        };

        mi_can_id can_id;
        uint8_t len;
        uint8_t __pad;
        uint8_t __res0;
        uint8_t len8_dlc;
        uint8_t data[CAN_MAX_DLEN];
    };

    enum class RunMode {
        DYNAMIC = 0,
        POSITION = 1,
        VELOCITY = 2,
        CURRENT = 3,
    };

    MiMotor(const std::unordered_map<std::string, std::string> &motor_param,
            uint8_t host_id);

    ~MiMotor() = default;

    can_frame motor_runmode_frame() const;
    can_frame motor_enable_frame() const;
    can_frame motor_disable_frame() const;
    can_frame motor_limit_frame() const;
    can_frame motor_loc_kp_frame() const;
    can_frame motor_spd_kp_frame() const;
    can_frame motor_spd_ki_frame() const;
    can_frame motor_dyn_frame(double position, double velocity, double effort) const;
    can_frame motor_pos_frame(double position) const;
    can_frame motor_vel_frame(double velocity) const;

    can_frame motor_wr_param_frame(uint16_t index, float value) const;
    can_frame motor_wr_param_frame(uint16_t index, uint8_t value) const;

    void set_motor_feedback(const mi_can_frame &can_msg);

    std::tuple<double, double, double> get_motor_feedback() const;

    RunMode get_run_mode() const { return run_mode_; }

  private:
    // Motor information
    std::string motor_model_;
    uint8_t mi_motor_id_;
    uint8_t host_id_;

    // Motor feedback
    double position_{0.0};
    double velocity_{0.0};
    double torque_{0.0};

    // Motor control parameters
    uint16_t Kp_raw_{0};
    uint16_t Kd_raw_{0};
    RunMode run_mode_;
    float limit_cur_{0.0};
    float limit_spd_{0.0};
    float loc_kp_{0.0};
    float spd_kp_{0.0};
    float spd_ki_{0.0};
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_DRIVER__MI_MOTOR_DRIVER_HPP_
