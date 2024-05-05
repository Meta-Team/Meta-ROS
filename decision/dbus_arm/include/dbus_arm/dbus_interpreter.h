#ifndef DBUS_INTERPRETER_H
#define DBUS_INTERPRETER_H

#include "operation_interface/msg/dbus_control.hpp"
#include "behavior_interface/msg/end_vel.hpp"
#include <thread>

#define PERIOD 10 // ms

using operation_interface::msg::DbusControl;
using behavior_interface::msg::EndVel;

class DbusInterpreter
{
public:
    DbusInterpreter(double linear, double angular, double deadzone);

    ~DbusInterpreter();

    void input(const DbusControl msg);

    EndVel::SharedPtr get_end_vel() const;

private:
    double ls_x, ls_y, rs_x, rs_y;

    double linear_v, angular_v, deadzone;

    EndVel::SharedPtr end_vel_;

    void apply_deadzone(double &val);

    void curb(double &val, double max_val);

    std::thread update_thread;

    void update();
};

#endif // DBUS_INTERPRETER_H