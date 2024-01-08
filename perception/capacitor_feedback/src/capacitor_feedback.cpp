#include "capacitor_feedback/capacitor_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "capacitor_interface/srv/capacitor_state.hpp"
#include <memory>

#define FEEDBACK_R 1 // ms

class CapacitorFeedback : public rclcpp::Node
{
public:
    CapacitorFeedback() : Node("CapacitorFeedback")
    {
        capa_srv_ = this->create_service<capacitor_interface::srv::CapacitorState>(
            "capacitor_state", [this](const std::shared_ptr<capacitor_interface::srv::CapacitorState::Request> req,
                                      std::shared_ptr<capacitor_interface::srv::CapacitorState::Response> res){
                capa_srv_callback(req, res);
            });
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(FEEDBACK_R), [this](){
                timer_callback();
            });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::ServiceBase::SharedPtr capa_srv_;
    std::unique_ptr<CapacitorDriver> capacitor_driver_;

    void timer_callback()
    {
        capacitor_driver_->get_frame();
        capacitor_driver_->process_rx();
    }

    void capa_srv_callback(const std::shared_ptr<capacitor_interface::srv::CapacitorState::Request> /*req*/,
                           std::shared_ptr<capacitor_interface::srv::CapacitorState::Response> res)
    {
        res->input_current = capacitor_driver_->input_current;
        res->input_voltage = capacitor_driver_->input_voltage;
        res->capacitor_voltage = capacitor_driver_->capacitor_voltage;
        res->output_power = capacitor_driver_->output_power;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}