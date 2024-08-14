#ifndef META_HARDWARE__MOTOR_DRIVER__BRT_ENCODER_DRIVER_HPP_
#define META_HARDWARE__MOTOR_DRIVER__BRT_ENCODER_DRIVER_HPP_
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

class BrtEncoder {
  public:
    explicit BrtEncoder(const std::unordered_map<std::string, std::string> &motor_param);

    ~BrtEncoder() = default;

    void set_encoder_feedback(const can_frame &can_msg);

    double get_encoder_feedback() const;

  private:
    // Motor information
    std::string encoder_model_;
    uint8_t brt_encoder_id_;
    uint32_t resolution_;

    // Motor feedback
    double position_{0.0};
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_DRIVER__BRT_ENCODER_DRIVER_HPP_
