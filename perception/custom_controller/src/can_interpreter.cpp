#include "custom_controller/can_interpreter.h"
#include "custom_controller/can_driver.hpp"
#include <cstdint>
#include <linux/can.h>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include "rclcpp/rclcpp.hpp"

void multiply_matrices(const double A[4][4], const double B[4], double C[4])
{
    // Initialize the result matrix C to 0
    for (int i = 0; i < 4; i++) {
        C[i] = 0;
    }

    // Perform the multiplication A * B and store it in C
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i] += A[i][j] * B[j];
        }
    }
}

CanInterpreter::CanInterpreter()
{
    this->can_driver_ = std::make_unique<CanDriver>(0);
    this->rx_thread = std::thread(&CanInterpreter::rx_loop, this);
    this->calc_thread = std::thread(&CanInterpreter::calc_loop, this);

    // this->msg.x_vel = 1.0;
}

CanInterpreter::~CanInterpreter()
{
    if (rx_thread.joinable()) rx_thread.join();
    if (calc_thread.joinable()) calc_thread.join();
}

void CanInterpreter::rx_loop()
{
    while (rclcpp::ok())
    {
        can_frame frame;
        
        can_driver_->get_frame(frame);
        // Reader program authored by Zhuohao Xu
        // if (frame.can_id == 0x301)
        // {
        // float position;
        // position = process_rx(frame);
        // RCLCPP_INFO(rclcpp::get_logger("custom_controller"),"frame: %f", position);
        // }
        // RCLCPP_INFO(rclcpp::get_logger("custom_controller"),"frame: %d", frame.data[0]);
        // RCLCPP_INFO(rclcpp::get_logger("custom_controller"),"frame");
        angle[frame.can_id - 0x301] = process_rx(frame);
        // RCLCPP_INFO(rclcpp::get_logger("custom_controller"),"frame: %f, %f, %f", angle[0], angle[1], angle[2]);
}

}

CC CanInterpreter::get_msg()
{
    return msg;
}

// cb
// r2r custom_controller custom_controller
// r2r plotjuggler plotjuggler


// Reader program authored by Zhuohao Xu
double CanInterpreter::process_rx(can_frame rx_frame)
{
    float received_value;
    memcpy(&received_value, rx_frame.data, 4);
    return static_cast<double>(received_value);
}



// Postion solver series functions authored by Zhuohao Xu
void CanInterpreter::calculate_rotation_matrix(double angleX, double angleY, double angleZ, double R[4][4])
{
    double Rx[3][3] = {
        {1, 0, 0},
        {0, cos(angleX), -sin(angleX)},
        {0, sin(angleX), cos(angleX)}
    };
    
    double Ry[3][3] = {
        {cos(angleY), 0, sin(angleY)},
        {0, 1, 0},
        {-sin(angleY), 0, cos(angleY)}
    };
    
    double Rz[3][3] = {
        {cos(angleZ), -sin(angleZ), 0},
        {sin(angleZ), cos(angleZ), 0},
        {0, 0, 1}
    };

    double temp[3][3] = {{0}};  // Temporary matrix to store intermediate results

    // Multiply Rz and Ry
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                temp[i][j] += Rz[i][k] * Ry[k][j];
            }
        }
    }

    // Multiply result with Rx and store in R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                R[i][j] += temp[i][k] * Rx[k][j];
            }
        }
    }
}

void CanInterpreter::position_calculation()
{
    memcpy(&result_old, result, 4);
    memcpy(&rotation_old, &angle[6], 3 * sizeof(double));

    calculate_rotation_matrix(angle[3], angle[4], angle[5], T_first);
    calculate_rotation_matrix(angle[0], angle[1], angle[2], T_zero);
    multiply_matrices(T_first, End_position, result);
    // multiply_matrices(T_zero, temp, result);
    // msg.x_vel = (result[0]-result_old[0])/DT;
    // msg.y_vel = (result[1]-result_old[1])/DT;
    // msg.z_vel = (result[2]-result_old[2])/DT;
    msg.x_vel = result[0];
    msg.y_vel = result[1];
    msg.z_vel = result[2];


    // msg.pitch_vel = (angle[6] - rotation_old[0])/DT;
    // msg.yaw_vel = (angle[7] - rotation_old[1])/DT;
    // msg.roll_vel = (angle[8] - rotation_old[2])/DT;
    msg.pitch_vel = angle[6];
    msg.yaw_vel = angle[7];
    msg.roll_vel = angle[8];
}

void CanInterpreter::calc_loop()
{
    while (rclcpp::ok())
    {
        position_calculation();
        rclcpp::sleep_for(std::chrono::milliseconds(CALC_PERIOD));
        // RCLCPP_INFO(rclcpp::get_logger("custom_controller"),"frame: %f, %f, %f", result[0], result[1], result[2]);
    }
}