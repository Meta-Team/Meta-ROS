#include "rclcpp/rclcpp.hpp"
#include "dji_controller/dji_driver.h"

#include "motor_interface/msg/dji_goal.hpp"
#include "motor_interface/srv/motor_present.hpp"
#include <cmath>
#include <cstdint>
#include <linux/can.h>

class DjiController : public rclcpp::Node
{
public:
    DjiController() : Node("DjiController")
    {
        motor_init();
        sub_ = this->create_subscription<motor_interface::msg::DjiGoal>(
            "motor_goal", 10, [this](const motor_interface::msg::DjiGoal::SharedPtr msg){
                goal_callback(msg);
            });
        cli_ = this->create_client<motor_interface::srv::MotorPresent>("motor_present");
    }

private:
    rclcpp::Subscription<motor_interface::msg::DjiGoal>::SharedPtr sub_;
    rclcpp::Client<motor_interface::srv::MotorPresent>::SharedPtr cli_;
    int motor_count;
    DjiDriver* driver_[8];
    can_frame tx_frame;
    std::vector<double> p2v_kps, p2v_kis, p2v_kds;
    std::vector<double> v2c_kps, v2c_kis, v2c_kds;

    void goal_callback(const motor_interface::msg::DjiGoal::SharedPtr msg)
    {
        auto request = std::make_shared<motor_interface::srv::MotorPresent::Request>();
        for (int i = 0; i < 4; i++) request->motor_id.push_back(msg->motor_id[i]);
        auto result = cli_->async_send_request(request);
        for (int i = 0; i < 4; i++)
        {
            int id = msg->motor_id[i];
            // update pos and vel info
            driver_[id]->update_pos(result.get()->present_pos[i]);
            driver_[id]->update_vel(result.get()->present_vel[i]);
            driver_[id]->set_goal(msg->goal_pos[i], msg->goal_vel[i]);
            // write the tx_frame
            driver_[id]->write_frame(tx_frame);
        }
        // send the tx_frame
        DjiDriver::send_frame(tx_frame);
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);
        std::vector<int64_t> motor_modes;
        motor_modes = this->declare_parameter("motor_types", motor_modes);

        p2v_kps = this->declare_parameter("p2v_kps", p2v_kps);
        p2v_kis = this->declare_parameter("p2v_kis", p2v_kis);
        p2v_kds = this->declare_parameter("p2v_kds", p2v_kds);

        v2c_kps = this->declare_parameter("v2c_kps", v2c_kps);
        v2c_kis = this->declare_parameter("v2c_kis", v2c_kis);
        v2c_kds = this->declare_parameter("v2c_kds", v2c_kds);

        // initialize the drivers
    }

    void update_pid()
    {
        p2v_kps = this->get_parameter("kps").as_double_array();
        p2v_kis = this->get_parameter("kis").as_double_array();
        p2v_kds = this->get_parameter("kds").as_double_array();
        for (int i = 0; i < motor_count; i++)
        {
            driver_[i]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            driver_[i]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DjiController>());
    rclcpp::shutdown();
    return 0;
}