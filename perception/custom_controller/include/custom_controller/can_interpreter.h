#ifndef CAN_INTERPRETER_H
#define CAN_INTERPRETER_H

#include "custom_controller/can_driver.hpp"
#include <memory>
#include <thread>

#include "operation_interface/msg/custom_controller.hpp"

using CC = operation_interface::msg::CustomController;

#define CALC_PERIOD 10 // ms
#define DT CALC_PERIOD / 1000.0 // s

class CanInterpreter
{
public:
    CanInterpreter();

    ~CanInterpreter();

    CC get_msg();

private:
    std::unique_ptr<CanDriver> can_driver_;

    std::thread rx_thread;
    std::thread calc_thread;

    CC msg{};

    void rx_loop();
    void calc_loop();

    double process_rx(can_frame rx_frame);

    // angle[0,2]: First x,y,z; angle[3,5]: Second x,y,z; angle[6,8]: Third  x,y,z
    // meta_c_3                 meta_c_1                  Cban01
    double angle[9];
    double rotation_old[3];

    // End position of the handle(controller)
    double position[3];


    //To Zhizhan: length0 = base->startpoint, length[1] = starpoint->First pivitol, etc.
    double length0[3] = {1,0,0};
    double length1[3] = {1,0,0};
    double length2[3] = {1,0,0};

    double T_second[4][4] = {
        {0, 0, 0, length2[0]},
        {0, 0, 0, length2[1]},
        {0, 0, 0, length2[2]},
        {0, 0, 0, 1},
    };

    double End_position[4] = {
        length2[0],
        length2[1],
        length2[2],
        1,
    };

    double T_first[4][4] = {
        {0, 0, 0, length1[0]},
        {0, 0, 0, length1[1]},
        {0, 0, 0, length1[2]},
        {0, 0, 0, 1},
    };

    double T_zero[4][4] = {
        {0, 0, 0, length0[0]},
        {0, 0, 0, length0[1]},
        {0, 0, 0, length0[2]},
        {0, 0, 0, 1},
    };
    double result[4];
    double result_old[4];
    double temp[4];

    void calculate_rotation_matrix(double angleX, double angleY, double angleZ, double R[4][4]);
    void position_calculation();
};

#endif