#include <cmath>
#include <cstdint>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <tuple>

#include "angles/angles.h"
#include "meta_hardware/motor_driver/dji_motor_driver.hpp"

namespace meta_hardware {
DjiMotor::DjiMotor(const std::string &motor_model, uint32_t dji_motor_id)
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

uint32_t DjiMotor::get_dji_motor_id() const { return dji_motor_id_; }
canid_t DjiMotor::get_tx_can_id() const { return tx_can_id_; }
canid_t DjiMotor::get_rx_can_id() const { return rx_can_id_; }
double DjiMotor::get_maximum_current() const { return maximum_current_; }
uint32_t DjiMotor::get_maximum_raw_effort() const {
    return maximum_raw_effort_;
}

void DjiMotor::set_motor_feedback(int16_t position_raw, int16_t velocity_raw,
                                  int16_t current_raw) {
    double position = position_raw / 8191.0 * (2 * M_PI);
    position_ += angles::shortest_angular_distance(position_, position);
    velocity_ = velocity_raw * M_PI / 30.0;
    current_ = current_raw / 16384.0 * maximum_current_;
}

std::tuple<double, double, double> DjiMotor::get_motor_feedback() const {
    return {position_, velocity_, current_};
}

} // namespace meta_hardware
