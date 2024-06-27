#ifndef PID_ALGORITHM_HPP
#define PID_ALGORITHM_HPP

#include <thread>

/**
 * @brief A PID algorithm class.
 */
class PidAlgorithm
{
public:
    /**
     * @brief The PID parameters.
     */
    class PidParam
    {
    public:
        double kp;
        double ki;
        double kd;

        PidParam(double kp, double ki, double kd)
            : kp(kp), ki(ki), kd(kd) {}
    };

    /**
     * @brief Construct a new object.
     * @param param The PID parameters.
     * @param dt The time interval for calculating the PID output.
     * @param max_output The maximum output value.
     * @param max_i The maximum integral value.
     */
    PidAlgorithm(PidParam param, int dt = 1,
        double max_output = std::numeric_limits<double>::max(),
        double max_i = std::numeric_limits<double>::max())
        : param(param)
    {
        this->dt = dt;

        this->max_i = max_i;
        this->max_output = max_output;

        running = true;
        calc_thread = std::thread(&PidAlgorithm::calc, this);
    }

    ~PidAlgorithm()
    {
        running = false;
        if (calc_thread.joinable()) calc_thread.join();
    }

    /**
     * @brief Start the PID algorithm.
     */
    void start()
    {
        active = true;
    }

    /**
     * @brief Stop the PID algorithm.
     * @note This will clear all stored values.
     */
    void stop()
    {
        active = false;

        // clear all stored values
        error = 0;
        prev_error = 0;
        proportional = 0;
        integral = 0;
        derivative = 0;
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
    bool running = true; ///< Whether the program is running.
    bool active = false; ///< Whether the PID algorithm is active.

    PidParam param;
    int dt;
    double max_i;
    double max_output;

    double feedback = 0;
    double target = 0;
    double output = 0;

    double error = 0;
    double prev_error = 0;
    double proportional = 0;
    double integral = 0;
    double derivative = 0;

    std::thread calc_thread; ///< The thread for calculating the PID output.

    /**
     * @brief Calculate the PID output.
     * @note This is the main thread for the PID algorithm.
     */
    void calc()
    {
        const double dt_seconds = dt / 1000.0;

        while (running)
        {
            if (!active)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(dt));
                continue;
            }

            error = target - feedback;
            proportional = param.kp * error; // P
            integral += param.ki * error * dt_seconds; curb(integral, max_i, -max_i); // I
            derivative = param.kd * (error - prev_error) / dt_seconds; // D

            prev_error = error;

            output = proportional + integral + derivative;
            curb(output, max_output, -max_output);

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