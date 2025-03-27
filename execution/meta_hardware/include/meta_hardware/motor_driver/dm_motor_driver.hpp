#ifndef META_HARDWARE__MOTOR_DRIVER__DM_MOTOR_DRIVER_HPP_
#define META_HARDWARE__MOTOR_DRIVER__DM_MOTOR_DRIVER_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <tuple>
#include <unordered_map>

#include <linux/can.h>

#include "angles/angles.h"

namespace meta_hardware {
class DmMotor {
  public:
    DmMotor(const std::unordered_map<std::string, std::string> &motor_param,
                 uint8_t master_id);

    ~DmMotor() = default;

    enum RunMode {
        MIT,
        POSITION,
        VELOCITY,
    };

    uint32_t get_dm_motor_id() const;
    uint32_t get_tx_can_id() const;
    uint32_t get_rx_can_id() const;
    
    can_frame motor_enable_frame() const;
    can_frame motor_disable_frame() const;
    can_frame motor_save_initial_frame() const;
    can_frame motor_clear_error_frame() const;
    can_frame motor_mit_frame(double position, double velocity, double effort) const;
    can_frame motor_pos_frame(double position) const;
    can_frame motor_vel_frame(double velocity) const;

    void set_motor_feedback(const can_frame &can_msg);
    std::tuple<double, double, double> get_motor_feedback() const;

    RunMode get_run_mode() const { return run_mode_; }

  private:
    // helper function
    uint32_t double_to_raw(float value, float max, float min, uint8_t bit) const;

    // Motor information
    std::string motor_model_;
    uint32_t dm_motor_id_;
    RunMode run_mode_;
    uint32_t tx_can_id_;
    uint32_t rx_can_id_;

    uint32_t master_id_;

    double max_vel_;
    double max_pos_;
    double max_effort_;

    double Kp_;
    double Kd_;
    uint16_t Kp_raw_;
    uint16_t Kd_raw_;

    // Motor feedback
    uint8_t error_code_{0};
    uint8_t id_{0};
    double position_{0.0};
    double velocity_{0.0};
    double toruqe_{0.0};
    uint8_t temperature_mos_{0};
    uint8_t temperature_rotor_{0};

};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_DRIVER__DM_MOTOR_DRIVER_HPP_
