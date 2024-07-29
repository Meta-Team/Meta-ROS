#ifndef METAV_HARDWARE__MOTOR_DRIVER__DJI_MOTOR_DRIVER_HPP_
#define METAV_HARDWARE__MOTOR_DRIVER__DJI_MOTOR_DRIVER_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

#include "metav_hardware/motor_driver/can_motor_driver.hpp"

namespace metav_hardware {
class DjiMotor : public CanMotorDriver {
  public:
    DjiMotor(const std::string &motor_model, uint32_t dji_motor_id)
        : motor_model_(motor_model), dji_motor_id_(dji_motor_id) {
        if (motor_model_ == "GM6020") {
            tx_can_id_ = (dji_motor_id_ <= 4) ? 0x1FF : 0x2FF;
            rx_can_id_ = 0x204 + dji_motor_id_;
            maximum_current_ = 3.0;
            maximum_raw_effort_ = 25000;
        } else if (motor_model_ == "M3508") {
            tx_can_id_ = (dji_motor_id_ <= 4) ? 0x200 : 0x1FF;
            rx_can_id_ = 0x200 + dji_motor_id_;
            maximum_current_ = 20.0;
            maximum_raw_effort_ = 16384;
        } else if (motor_model_ == "M2006") {
            tx_can_id_ = (dji_motor_id_ <= 4) ? 0x200 : 0x1FF;
            rx_can_id_ = 0x200 + dji_motor_id_;
            maximum_current_ = 10.0;
            maximum_raw_effort_ = 10000;
        } else {
            throw std::runtime_error("Unknown motor model: " + motor_model_);
        }
    }

    ~DjiMotor() override = default;

    inline std::string get_motor_model() const { return motor_model_; }
    inline uint32_t get_dji_motor_id() const { return dji_motor_id_; }
    inline uint32_t get_tx_can_id() const { return tx_can_id_; }
    inline uint32_t get_rx_can_id() const { return rx_can_id_; }
    inline double get_maximum_current() const { return maximum_current_; }
    inline uint32_t get_maximum_raw_effort() const {
        return maximum_raw_effort_;
    }

    inline void set_motor_feedback(int16_t position_raw, int16_t velocity_raw,
                                   int16_t current_raw) {
        position_ = position_raw / 8191.0 * (2 * M_PI);
        velocity_ = velocity_raw * M_PI / 30.0;
        current_ = current_raw / 16384.0 * maximum_current_;
    }

    inline std::tuple<double, double, double> get_motor_feedback() const {
        return {position_, velocity_, current_};
    }

  private:
    // Motor information
    std::string motor_model_;
    uint32_t dji_motor_id_;
    uint32_t tx_can_id_;
    uint32_t rx_can_id_;
    uint32_t maximum_raw_effort_;
    double maximum_current_;

    // Motor feedback
    double position_{std::numeric_limits<double>::quiet_NaN()};
    double velocity_{std::numeric_limits<double>::quiet_NaN()};
    double current_{std::numeric_limits<double>::quiet_NaN()};
};

} // namespace metav_hardware

#endif // METAV_HARDWARE__MOTOR_DRIVER__DJI_MOTOR_DRIVER_HPP_
