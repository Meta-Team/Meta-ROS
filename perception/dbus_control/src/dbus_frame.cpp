#include "dbus_control/dbus_frame.hpp"

float DbusFrame::linear_mapping(int raw, int raw_min, int raw_max, float min, float max)
{
    auto slope = (max - min) / (raw_max - raw_min);
    auto data = slope * (raw - raw_min) + min;
    return data;
}

DbusFrame::DbusFrame(const std::vector<uint8_t> &frame)
{
    std::copy(frame.begin(), frame.end(), reinterpret_cast<uint8_t *>(&interpreted));
}

operation_interface::msg::DbusControl DbusFrame::msg()
{
    operation_interface::msg::DbusControl msg;
    // rc
    static auto stick_mapping = [this](int raw) {
        return - linear_mapping(raw, CH_MIN, CH_MAX, -1, 1);
    };
    msg.ls_x = stick_mapping(interpreted.rc.ch3);
    msg.ls_y = stick_mapping(interpreted.rc.ch2);
    msg.rs_x = stick_mapping(interpreted.rc.ch1);
    msg.rs_y = stick_mapping(interpreted.rc.ch0);
    msg.lsw = interpreted.rc.s1;
    msg.rsw = interpreted.rc.s2;
    // MY_TODO: mouse
    // MY_TODO: keyboard
    return msg;
}