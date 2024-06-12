#ifndef PID_ALGORITHM_HPP
#define PID_ALGORITHM_HPP

#include <thread>

class PidAlgorithm
{
public:
    /**
     * @brief Construct a new PIDAlgorithm object.
     * @param kp The proportional gain of the PID controller.
     * @param ki The integral gain of the PID controller.
     * @param kd The derivative gain of the PID controller.
     * @param dt The time interval for calculating the PID output.
     */
    PidAlgorithm(double kp, double ki, double kd, int dt = 1,
        double max_i = std::numeric_limits<double>::max())
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;

        this->dt = dt;

        this->max_i = max_i;

        running = true;
        calc_thread = std::thread(&PidAlgorithm::calc, this);
    }

    ~PidAlgorithm()
    {
        running = false;
        if (calc_thread.joinable()) calc_thread.join();
    }

    void set_target(double x)
    {
        target = x;
    }

    void set_feedback(double x)
    {
        feedback = x;
    }

    [[nodiscard]] double get_output() const
    {
        return output;
    }

    [[nodiscard]] double get_feedback() const
    {
        return feedback;
    }

private:
    bool running = true; ///< Whether the PID algorithm is running.

    double kp, ki, kd;
    int dt;
    double max_i;

    double feedback = 0;
    double target = 0;
    double output = 0;

    std::thread calc_thread; ///< The thread for calculating the PID output.

    /**
     * @brief Calculate the PID output.
     * @note This is the main thread for the PID algorithm.
     */
    void calc()
    {
        double error = 0;
        double prev_error = 0;
        double proportional = 0;
        double integral = 0;
        double derivative = 0;

        const double dt_seconds = dt / 1000.0;

        while (running)
        {
            error = target - feedback;
            proportional = kp * error; // P
            integral += ki * error * dt_seconds; curb(integral, max_i, -max_i); // I
            derivative = kd * (error - prev_error) / dt_seconds; // D

            prev_error = error;

            output = proportional + integral + derivative;

            std::this_thread::sleep_for(std::chrono::milliseconds(dt));
        }
    }

    void curb(double &val, double max, double min)
    {
        if (val > max)
            val = max;
        else if (val < min)
            val = min;
    }
};

#endif // PID_ALGORITHM_HPP