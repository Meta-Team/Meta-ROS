#ifndef META_HARDWARE__MOTOR_DRIVER__DJI_MOTOR_DRIVER_HPP_
#define META_HARDWARE__MOTOR_DRIVER__DJI_MOTOR_DRIVER_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

#include "angles/angles.h"

namespace meta_hardware {
class DjiMotor {
  public:
    DjiMotor(const std::string &motor_model, uint32_t dji_motor_id);

    ~DjiMotor() = default;

    uint32_t get_dji_motor_id() const;
    uint32_t get_tx_can_id() const;
    uint32_t get_rx_can_id() const;
    double get_maximum_current() const;
    uint32_t get_maximum_raw_effort() const;

    void set_motor_feedback(int16_t position_raw, int16_t velocity_raw,
                            int16_t current_raw);
    std::tuple<double, double, double> get_motor_feedback() const;

  private:
    // Motor information
    std::string motor_model_;
    uint32_t dji_motor_id_;
    uint32_t tx_can_id_;
    uint32_t rx_can_id_;
    uint32_t maximum_raw_effort_;
    double maximum_current_;

    // Motor feedback
    double position_{0.0};
    double velocity_{0.0};
    double current_{0.0};
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_DRIVER__DJI_MOTOR_DRIVER_HPP_
