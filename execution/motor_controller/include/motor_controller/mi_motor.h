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

#define TX_FREQ 10

#define MASTER_ID 0 // must be zero
// or the motor will keep sending feedback frames once enabled
// and it cannot be disabled

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
    MiMotor(const string& rid, int hid, string type, string port);

    ~MiMotor();

    void set_goal(double goal_pos, double goal_vel, double goal_tor) override;

    void set_param(double p2v_kp, double p2v_ki, double p2v_kd,
        double v2t_kp, double v2t_ki, double v2t_kd) override;

    [[nodiscard]] std::tuple<double, double, double> get_state() override;

    void print_info() override;

private:
    static umap<int, unique_ptr<CanDriver>> can_drivers; // {port, driver}
    static umap<int, thread> rx_threads; // {port, thread}
    static umap<int, can_frame> rx_frames; // {port, frame}

    static umap<int, umap<int, MiMotor*>> instances; // {port, {hid, instance}}

    struct TxMsg
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

    TxMsg control_msg; // The control message to be sent.

    double position; // The current position of the motor.
    double velocity; // The current velocity of the motor.
    double torque; // The current torque of the motor.
    int port; // The port number of the CAN bus.

    double kp;
    double kd;

    thread tx_thread;

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
     * @brief Create a CAN port when no port is available.
     * @param port The port number.
     * @note This create a new feedback loop thread if the port is not in the array.
     */
    static void create_port(int port);

    /**
     * @brief Destroy the CAN port when no motor is using it.
     * @param port The port number.
     * @note This function joins the threads for receiving and transmitting data.
     */
    static void destroy_port(int port);

    /**
     * @brief Sends a message to the motor.
     * @param msg The message to be sent.
     * @note This first converts the message to a CAN frame before sending it.
     */
    void tx(TxMsg msg);

    /**
     * @brief The transmission loop for sending data to the motor.
     */
    void tx_loop();

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