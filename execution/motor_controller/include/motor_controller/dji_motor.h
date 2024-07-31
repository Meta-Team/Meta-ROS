#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "motor_driver.h"
#include <linux/can.h>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>

using std::thread;
using std::unique_ptr;

#define umap std::unordered_map

#define T_MAX 20
#define V_MAX 1000
#define CALC_FREQ 1
#define TX_FREQ 2
#define JAMMED_THRESHOLD 0.3 // s

class DjiMotor : public MotorDriver
{
public:
    /**
     * @brief Construct a new DjiMotor object.
     * @param rid The resource ID of the motor.
     * @param hid The hardware ID of the motor.
     * @param type The type of the motor.
     * @param port The port number of the CAN bus.
     * @param cali The calibration direction of the motor.
     */
    DjiMotor(const string& rid, int hid, string type, string port, int cali);

    /**
     * @brief Destroy the DjiMotor object.
     * Join the threads for calculating PID control and calibrating the motor.
     */
    ~DjiMotor();

    void set_goal(double goal_pos, double goal_vel, double goal_tor) override;

    void set_param(double p2v_kp, double p2v_ki, double p2v_kd,
        double v2t_kp, double v2t_ki, double v2t_kd) override;

    [[nodiscard]] std::tuple<double, double, double> get_state() override;

    void print_info() override;

private:
    enum MotorType
    {
        M3508 = 0,
        M6020 = 1,
        M2006 = 2,
    };

    /**
     * @brief A CAN driver along with rx and tx frames.
     */
    class CanPort
    {
    private:
        std::unique_ptr<CanDriver> can_driver_; /**< Pointer to the CAN driver instance. */

        /**
        * @brief Initialize a CAN frame.
        * @param frame_id The ID of the frame.
        * @return A pointer to the CAN frame.
        */
        static can_frame init_frame(int frame_id)
        {
            can_frame frame;
            frame.can_id = frame_id;
            frame.can_dlc = 8;
            for (auto& data : frame.data) data = 0;
            return frame;
        }

    public:
        can_frame tx_frame_200, tx_frame_1ff, tx_frame_2ff; /**< CAN frames for transmitting data. */
        can_frame rx_frame; /**< CAN frame for receiving data. */

        /**
        * @brief Constructor for the CanPort class.
        * Initialize the CAN driver and CAN frames.
        * @param port The port number to use for the CAN driver.
        */
        CanPort(int port)
        {
            can_driver_ = std::make_unique<CanDriver>(port);
            tx_frame_200 = init_frame(0x200);
            tx_frame_1ff = init_frame(0x1ff);
            tx_frame_2ff = init_frame(0x2ff);
            rx_frame = init_frame(0);
        }

        /**
        * @brief Transmit data to the CAN bus.
        */
        void tx()
        {
            can_driver_->send_frame(tx_frame_200);
            can_driver_->send_frame(tx_frame_1ff);
            can_driver_->send_frame(tx_frame_2ff);
        }

        /**
        * @brief Receive data from the CAN bus.
        */
        void rx()
        {
            can_driver_->get_frame(rx_frame);
        }
    };

    /**
     * @brief Represents the data of a motor.
     */
    struct MotorData
    {
    public:
        double torque;
        double velocity; // rad/s
        double position; // cumulative position, rad

        /**
         * @brief Construct a new MotorData object.
         * Set all the data to zero.
         */
        MotorData() : torque(0), velocity(0), position(0) {}

        /**
         * @brief Update the cumulative position of the motor.
         * @param pos The new feedback position.
         */
        void update_pos(double pos)
        {
            position += min_error(pos, position);
        }

    private:
        /**
         * @brief Calculate the minimum error in (-M_PI, M_PI] between two angles.
         * Similar to `a - b`, but in the range (-M_PI, M_PI].
         * @param a The first angle.
         * @param b The second angle.
         * @return The minimum error in (-M_PI, M_PI].
         */
        double min_error(double a, double b)
        {
            double diff = a - b;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            return diff;
        }
    };

    /**
     * @brief Represents the parameters of a PID controller.
     */
    struct PidParam
    {
    public:
        double kp;
        double ki;
        double kd;

        PidParam(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {}
    };

    /**
     * @brief Represents the output of a PID controller.
     */
    struct PidOutput
    {
    public:
        double p;
        double i;
        double d;

        double sum() { return p + i + d; }

        PidOutput() : p(0.0), i(0.0), d(0.0) {}
    };

    static umap<int, unique_ptr<CanPort>> can_ports; // {port, ptr}
    static umap<int, std::thread> rx_threads; // {port, thread}
    static umap<int, std::thread> tx_threads; // {port, thread}

    static umap<int, umap<int, DjiMotor*>> instances; // {port, {fb_id, instance}}

    uint8_t port; /**< CAN port number. */
    MotorType motor_type; /**< Type of the motor. */
    PidParam p2v_prm, v2t_prm; /**< PID parameters for position-to-velocity and velocity-to-torque conversion. */
    PidOutput p2v_out, v2t_out; /**< PID outputs for position-to-velocity and velocity-to-torque conversion. */

    MotorData present_data{}; /**< Data representing the current state of the motor. */

    double vel_error{}; /**< Error in velocity. */
    double pos_error{}; /**< Error in position. */
    double goal_pos = NaN; /**< Desired position of the motor. Initially set to NaN to ensure that motor doesn't attempt to adjust to 0 position.*/
    double goal_vel{}; /**< Desired velocity of the motor. */
    double torque{}; /**< Current or voltage value of the motor. */
    double zero = 0.0; /**< Zero position of the motor. */

    bool ready; /**< Flag to indicate if the motor is ready to receive commands. */

    std::thread calc_thread; /**< Thread for calculating PID control. */
    std::thread cali_thread; /**< Thread for calibrating the motor. */

    /**
     * @brief Convert velocity to current or voltage using PID control.
     * This depends on member variables `goal_vel`, `present_data.velocity`.
     * The result is stored in member variable `torque`.
     */
    void vel2torque();

    /**
     * @brief Convert position to velocity using PID control.
     * This depends on member variables `goal_pos`, `present_data.position`.
     * The result is stored in member variable `goal_vel`.
     * @note To realize position control, this function should be executed before vel2current().
     */
    void pos2velocity();

    /**
     * @brief Calculate the feedback ID according to the motor type and hid.
     * @return The feedback ID of the motor.
     */
    int calc_fb_id();

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
     * @brief Receive loop for processing received data.
     * @param port The port number of the CAN bus.
     * @note Each port should have a separate thread for receiving data.
     */
    static void rx_loop(int port);

    /**
     * @brief Transmit loop for sending data to the motor.
     * @param port The port number of the CAN bus.
     * @note Each port should have a separate thread for transmitting data.
     */
    static void tx_loop(int port);

    /**
     * @brief Calibration loop for measuring the zero position of the motor.
     * @param dir The direction of calibration.
     * @note This function is executed once when the motor is initialized.
     */
    void cali_loop(int dir);

    /**
     * @brief Calculate the PID control.
     * This function is executed in a separate thread.
     * @note Each motor should have a separate thread for calculating PID control.
     */
    void calc_loop(); /**< Loop for calculating PID control. */

    /**
     * @brief Calculate and write the transmit frames.
     * PID control is executed in this function.
     * Position-to-velocity conversion is executed iff goal_pos is not zero.
     * @note This function should be executed before sending the transmit frame.
     */
    void calc_tx();

    /**
     * @brief Process the receive frame.
     * This updates the present data of the motor according to the rx_frame variable.
     * @note rx() should be executed before calling this function.
     */
    void process_rx();

    /**
     * @brief Limits the value of a variable to a specified range.
     * @param val Reference to the variable to be limited.
     * @param limit The upper and lower limit for the value.
     * @note Limit must be positive.
     */
    void curb(double &val, double limit);
};

#endif // DJI_MOTOR_H