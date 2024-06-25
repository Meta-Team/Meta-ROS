#ifndef MI_MOTOR_H
#define MI_MOTOR_H

#include "motor_driver.h"
#include "can_driver.hpp"
#include <array>
#include <cstdint>
#include <linux/can.h>
#include <thread>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>

using std::thread;
using std::unique_ptr;

#define umap std::unordered_map

#define MASTER_ID 1

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define IQ_REF_MIN -27.0f
#define IQ_REF_MAX 27.0f
#define SPD_REF_MIN -30.0f
#define SPD_REF_MAX 30.0f
#define LIMIT_TORQUE_MIN 0.0f
#define LIMIT_TORQUE_MAX 12.0f
#define CUR_FILT_GAIN_MIN 0.0f
#define CUR_FILT_GAIN_MAX 1.0f
#define LIMIT_SPD_MIN 0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_CUR_MIN 0.0f
#define LIMIT_CUR_MAX 27.0f

class MiMotor : public MotorDriver
{
public:
    MiMotor(const string& rid, int hid, string type, string port, int cali);

    ~MiMotor();

    void set_goal(double goal_pos, double goal_vel, double goal_cur) override;

    void set_param(double p2v_kp, double p2v_ki, double p2v_kd,
        double v2t_kp, double v2t_ki, double v2t_kd) override;

    [[nodiscard]] std::tuple<double, double, double> get_state() override;

    void print_info() override;

private:
    static umap<int, unique_ptr<CanDriver>> can_drivers; /**< A map from port number to pointers to the CAN port instances. */
    static umap<int, thread> rx_threads; /**< A map from port number to threads for the feedback loop. */
    static umap<int, can_frame> rx_frames; /**< A map from port number to receiving frames. */

    static umap<int, std::shared_ptr<MiMotor>> instances; /** A map from hid to shared pointers to all the motor instances. */

    struct Frame
    {
        struct [[gnu::packed]] ExtId
        {
            uint8_t id;
            uint16_t data;
            uint8_t mode : 5;
            uint8_t res : 3;
        };

        ExtId ext_id;
        std::array<uint8_t, 8> data;
    };

    Frame parsed_frame; // The frame to be sent.

    double position; // The current position of the motor.
    double velocity; // The current velocity of the motor.
    double torque; // The current torque of the motor.
    int port; // The port number of the CAN bus.

    double kp;
    double kd;

    /**
     * @brief A helper function for setting the goal.
     * @param pos The goal position.
     * @param vel The goal velocity.
     * @param tor The goal torque.
     * @param kp The proportional gain.
     * @param kd The derivative gain.
     */
    void goal_helper(float pos, float vel, float tor, float kp , float kd);

    /**
     * @brief Sets the port number of the CAN bus.
     * @param port The port number of the CAN bus.
     * @note Threads for receiving data are created for each port.
     */
    void set_port(int port);

    /**
     * @brief Sends the parsed frame to the motor.
     * @note This first converts the parsed frame to a can_frame and then sends it.
     */
    void tx();

    /**
     * @brief Calculates the id of the motor from the feedback frames.
     * @param frame The frame to be processed.
     * @return The id of the motor.
     */
    static int calc_id(const can_frame& frame);

    /**
     * @brief Processes the received frame.
     * @param frame The frame to be processed.
     */
    void process_rx(const can_frame& frame);

    /**
     * @brief Receive loop for processing received data.
     * @param port The port number of the CAN bus.
     * @note Each port should have a separate thread for receiving data.
     */
    static void rx_loop(int port);

    /**
     * @brief Starts the motor.
     */
    void start();

    /**
     * @brief Stops the motor.
     */
    void stop();

    /**
     * @brief Converts a floating point number to an unsigned integer.
     * @param x The floating point number to be converted.
     * @param x_min The minimum value of the floating point number.
     * @param x_max The maximum value of the floating point number.
     * @param bits The number of bits to be used for the conversion.
     * @return The converted unsigned integer.
     */
    static uint32_t float_to_uint(float x, float x_min, float x_max, int bits);

    /**
     * @brief Limits the value of a variable to a specified range.
     * @param val Reference to the variable to be limited.
     * @param limit The upper and lower limit for the value.
     * @note Limit must be positive.
     */
    void curb(double &val, double limit);
};

#endif // MI_MOTOR_H