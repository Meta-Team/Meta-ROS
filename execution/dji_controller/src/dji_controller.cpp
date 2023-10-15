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
    can_frame tx_frame_;
    std::vector<double> p2v_kps, p2v_kis, p2v_kds;
    std::vector<double> v2c_kps, v2c_kis, v2c_kds;

    void goal_callback(const motor_interface::msg::DjiGoal::SharedPtr msg)
    {
        auto request = std::make_shared<motor_interface::srv::MotorPresent::Request>();
        for (int i = 0; i < 4; i++) request->motor_id.push_back(msg->motor_id[i]);
        auto result = cli_->async_send_request(request);
        for (int i = 0; i < 4; i++) // update 4 present pos and vel
        {
            driver_[msg->motor_id[i]]->update_pos(result.get()->present_pos[i]);
            driver_[msg->motor_id[i]]->update_vel(result.get()->present_vel[i]);
        }
        for (int i = 0; i < 4; i++) // set 4 goal current
        {
            float current;
            DjiDriver::set_current(current, driver_[msg->motor_id[i]]->vel2current(msg->goal_vel[i]));
            DjiDriver::set_current(current, driver_[msg->motor_id[i]]->pos2current(msg->goal_pos[i]));
            std::uint16_t current_data = DjiDriver::float_to_uint(current, -I_MAX, I_MAX, 16);

            int id = msg->motor_id[i];
            if (id > 4) id -= 4; // id 5 is the same as id 1, etc.
            tx_frame_.data[2*id-2] = current_data >> 8; // id 1 is at index 0, id 2 is at index 2, etc.
            tx_frame_.data[2*id-1] = current_data & 0xff; // id 1 is at index 1, id 2 is at index 3, etc.
        }
        tx_frame_.can_id = 0x200;
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