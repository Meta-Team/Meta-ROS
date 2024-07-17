#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "motor_driver.h"
#include <linux/can.h>
#include <thread>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>

using namespace std;

#define umap std::unordered_map

#define TX_FREQ 5 // ms

#define START_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}
#define STOP_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}
#define SAVE_ZERO_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe}
#define CLEAR_ERROR_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}

// can be read from the debugging assistant
#define P_MAX 3.141593
#define V_MAX 30.0
#define T_MAX 10.0
#define P_MIN -3.141593
#define V_MIN -30.0
#define T_MIN -10.0

#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0

class DmMotor : public MotorDriver
{
public:
    DmMotor(const string& rid, const int hid, string type, string port);

    ~DmMotor();

    void set_goal(double goal_pos, double goal_vel, double goal_tor) override;

    void set_param(double p2v_kp, double p2v_ki, double p2v_kd,
        double v2t_kp, double v2t_ki, double v2t_kd) override;

    [[nodiscard]] tuple<double, double, double> get_state() override;

    void print_info() override;

private:
    static umap<int, unique_ptr<CanDriver>> can_drivers; // {port, driver}
    static umap<int, thread> rx_threads; // {port, thread}
    static umap<int, can_frame> rx_frames; // {port, frame}

    static umap<int, umap<int, DmMotor*>> instances; // {port, {hid, instance}}

    int port;
    can_frame tx_frame;
    thread tx_thread;

    float fb_pos, fb_vel, fb_tor;
    float kp, kd;

    /**
     * @brief A helper function for setting the frame to be sent.
     * @param pos The position.
     * @param vel The velocity.
     * @param tor The torque.
     * @param kp The proportional gain.
     * @param kd The derivative gain.
     */
    void set_frame(float pos, float vel, float tor, float kp, float kd);

    /**
     * @brief Send a CAN frame.
     * @param frame The frame to be sent.
     */
    void tx(can_frame frame) const;

    /**
     * @brief The transmission loop for the CAN driver.
     */
    void tx_loop();

    /**
     * @brief Turns on the motor.
     */
    void turn_on();

    /**
     * @brief Turns off the motor.
     */
    void turn_off();

    /**
     * @brief Set the mode to MIT.
     */
    void set_mode();

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
     * @brief Calculates the id of the motor from the feedback frames.
     * @param frame The frame to be processed.
     * @return The id of the motor.
     */
    static int calc_id(can_frame frame);

    /**
     * @brief The feedback loop for the CAN driver.
     */
    static void rx_loop(int port);

    /**
     * @brief Processes the received frame and updates the present data.
     */
    void process_rx();

    /**
     * @brief Converts an integer value to a floating-point value.
     * Provided by the DM motor.
     */
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    /**
     * @brief Converts a floating-point value to an integer value.
     * Provided by the DM motor.
     */
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

#endif // DM_MOTOR_H