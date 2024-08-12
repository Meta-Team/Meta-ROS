#ifndef UNITREE_MOTOR_H
#define UNITREE_MOTOR_H

#include "motor_driver.h"
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#define UPDATE_FREQ 10 // ms
#define CALI_TIMEOUT 3 // seconds
#define TRY_VEL 1.5
#define JAMMED_THRESHOLD 0.3 // s

class UnitreeMotor : public MotorDriver
{
public:
    UnitreeMotor(const string& rid, int hid, const string& type, const string& port, int cali);

    ~UnitreeMotor();

    void set_goal(double goal_pos, double goal_vel, double goal_tor) override;

    void set_param(double p2v_kp, double p2v_ki, double p2v_kd,
        double v2t_kp, double v2t_ki, double v2t_kd) override;

    [[nodiscard]] std::tuple<double, double, double> get_state() override;

    void print_info() override;

private:
    std::shared_ptr<SerialPort> port; ///< The serial port used to communicate with the motor.
    MotorType type; ///< The type of the motor.
    double kp, kd;
    MotorCmd goal_cmd; ///< The current goal command for the motor. Its symbols differ from the documentation.
    MotorData feedback_data; ///< The current feedback data for the motor. Its symbols differ from the documentation.

    std::thread control_thread; ///< The thread for the control loop.

    static std::unordered_map<std::string, std::shared_ptr<SerialPort>> serial_ports; ///< The serial ports used to communicate with the motor.

    /**
     * @brief Send the goal command and receive the feedback data.
     * This would send the goal_cmd and overwrite the feedback_data.
     * @note Used in the control loop.
     */
    void send_recv();

    /**
     * @brief Stop the motor.
     * This would send a zero command to the motor.
     */
    void stop();

    /**
     * @brief Control loop for the motor.
     * Used in the control_thread.
     * This would send the goal command and receive the feedback data at a fixed rate.
     */
    void control_loop();

    bool ready = false; ///< A flag to indicate if the motor has finished calibration, and is ready to receive commands.
    double zero = 0.0; ///< The zero position of the motor.

    std::thread cali_thread; ///< The thread for the calibration loop.

    /**
     * @brief Calibrate the motor.
     * Rotate the motor and try to find a position where it can be jammed physically.
     * Set that position as the zero position.
     * @note This and control_loop() should not be running at the same time.
     */
    void calibrate(int dir);
};

#endif // UNITREE_MOTOR_H