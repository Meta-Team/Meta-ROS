#include "capacitor_controller/capacitor_driver.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <chrono>

#define CONTROL_R 10 // control frame rate

class CapacitorController : public rclcpp::Node
{
public:
    CapacitorController() : Node("CapacitorController")
    {
        capa_driver = std::make_unique<CapacitorDriver>();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_R),
            std::bind(&CapacitorController::timer_callback, this));
    }

private:
    std::unique_ptr<CapacitorDriver> capa_driver;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        capa_driver->set_power(55.0f);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}