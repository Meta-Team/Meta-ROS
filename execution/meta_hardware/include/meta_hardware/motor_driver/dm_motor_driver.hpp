#ifndef META_HARDWARE__MOTOR_DRIVER__DM_MOTOR_DRIVER_HPP_
#define META_HARDWARE__MOTOR_DRIVER__DM_MOTOR_DRIVER_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

#include "angles/angles.h"

namespace meta_hardware {
class DmMotor {
  public:
    DmMotor(const std::string &motor_model, uint32_t dm_motor_id,std::string mode, 
            double max_vel, double max_pos, double max_effort);

    ~DmMotor() = default;

    enum DmMode {
        MIT,
        POS,
        VEL,
    };

    uint32_t get_dm_motor_id() const;
    uint32_t get_tx_can_id() const;
    uint32_t get_rx_can_id() const;
    DmMode get_mode() const { return mode_; }

    void set_motor_feedback(uint8_t error_code, uint8_t id, 
                            uint16_t position_raw, uint16_t velocity_raw, 
                            uint16_t effort_raw, uint8_t temperature_mos, 
                            uint8_t temperature_rotor);
    std::tuple<double, double, double> get_motor_feedback() const;

  private:
    // Motor information
    std::string motor_model_;
    uint32_t dm_motor_id_;
    DmMode mode_;
    uint32_t tx_can_id_;
    uint32_t rx_can_id_;

    double max_vel_;
    double max_pos_;
    double max_effort_;

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
