#ifndef DBUS_INTERPRETER_H
#define DBUS_INTERPRETER_H

#include <memory>
#include <thread>

#include "operation_interface/msg/dbus_control.hpp"
#include "behavior_interface/msg/move.hpp"

#define PERIOD 10 // ms

using operation_interface::msg::DbusControl;
using behavior_interface::msg::Move;

class DbusInterpreter
{
public:
    DbusInterpreter(double max_vel, double max_omega, double deadzone, double height_sens);

    ~DbusInterpreter();

    void input(const DbusControl::SharedPtr msg);

    Move::SharedPtr get_move() const;

    bool is_active() const { return active; }

private:
    bool active;

    double ls_x, ls_y, rs_x, rs_y, wheel;
    std::string lsw, rsw;

    double max_vel, max_omega, deadzone, height_sens;

    std::thread update_thread;

    Move::SharedPtr move_;

    void update();

    void apply_deadzone(double& val);

    void curb(double& val, double upper, double lower);
};

#endif // DBUS_INTERPRETER_H