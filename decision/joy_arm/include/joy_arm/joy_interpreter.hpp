#ifndef JOY_INTERPRETER_HPP
#define JOY_INTERPRETER_HPP

#include "sensor_msgs/msg/joy.hpp"
#include "behavior_interface/msg/end_vel.hpp"
#include <thread>

#define PERIOD 10 // ms

using sensor_msgs::msg::Joy;
using behavior_interface::msg::EndVel;

class JoyInterpreter
{
public:
    JoyInterpreter(double linear, double angular, double deadzone);

    ~JoyInterpreter();

    void input(const Joy::SharedPtr msg);

    EndVel::SharedPtr get_end_vel() const;

private:
    bool lb, rb, a, b, x, y, up, down, left, right;
    double lt, rt, ls_x, ls_y, rs_x, rs_y;

    double linear_v, angular_v, deadzone;

    EndVel::SharedPtr end_vel_;

    void apply_deadzone(double& val);

    void curb(double& val, double max_val);

    std::thread update_thread;

    void update();
};

#endif // JOY_INTERPRETER_HPP