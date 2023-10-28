#include "rclcpp/rclcpp.hpp"
#include "dji_controller/dji_driver.h"

#include "motor_interface/msg/dji_goal.hpp"
#include "motor_interface/srv/motor_present.hpp"
#include <cmath>
#include <cstdint>
#include <linux/can.h>
#include <vector>

class DjiController : public rclcpp::Node
{
public:
    DjiController() : Node("DjiController")
    {
        frame_init();
        motor_init();
        sub_ = this->create_subscription<motor_interface::msg::DjiGoal>(
            "motor_goal", 10, [this](const motor_interface::msg::DjiGoal::SharedPtr msg){
                goal_callback(msg);
            });
        cli_ = this->create_client<motor_interface::srv::MotorPresent>("motor_present");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this](){
            timer_callback();
        });
    }

private:
    rclcpp::Subscription<motor_interface::msg::DjiGoal>::SharedPtr sub_;
    rclcpp::Client<motor_interface::srv::MotorPresent>::SharedPtr cli_;
    rclcpp::TimerBase::SharedPtr timer_;
    int motor_count;
    DjiDriver* driver_[8];
    can_frame tx_frame1, tx_frame2;
    std::vector<double> p2v_kps, p2v_kis, p2v_kds;
    std::vector<double> v2c_kps, v2c_kis, v2c_kds;

    void timer_callback()
    {
        auto request = std::make_shared<motor_interface::srv::MotorPresent::Request>();
        request->motor_id.clear();
        for (int i = 0; i < motor_count; i++) request->motor_id.push_back(driver_[i]->motor_id);
        auto result = cli_->async_send_request(request);

        for (int i = 0; i < 4; i++)
        {
            // update pos and vel info
            driver_[i]->update_pos(result.get()->present_pos[i]);
            driver_[i]->update_vel(result.get()->present_vel[i]);
        }
    }
    
    void goal_callback(const motor_interface::msg::DjiGoal::SharedPtr msg)
    {
        update_pid();
        for (int i = 0; i < motor_count; i++)
        {
            driver_[i]->set_goal(msg->goal_pos[i], msg->goal_vel[i]);
            driver_[i]->write_frame(tx_frame1, tx_frame2);
        }
        DjiDriver::send_frame(tx_frame1, tx_frame2);
    }

    void frame_init()
    {
        tx_frame1.can_id = 0x200;
        tx_frame1.can_dlc = 8;
        for (int i = 0; i < 8; i++) tx_frame1.data[i] = 0x00;
        tx_frame2.can_id = 0x1ff;
        tx_frame2.can_dlc = 8;
        for (int i = 0; i < 8; i++) tx_frame2.data[i] = 0x00;
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);
        std::vector<int64_t> motor_ids;
        motor_ids = this->declare_parameter("motor_ids", motor_ids);
        std::vector<int64_t> motor_modes;
        motor_modes = this->declare_parameter("motor_types", motor_modes);

        p2v_kps = this->declare_parameter("p2v_kps", p2v_kps);
        p2v_kis = this->declare_parameter("p2v_kis", p2v_kis);
        p2v_kds = this->declare_parameter("p2v_kds", p2v_kds);

        v2c_kps = this->declare_parameter("v2c_kps", v2c_kps);
        v2c_kis = this->declare_parameter("v2c_kis", v2c_kis);
        v2c_kds = this->declare_parameter("v2c_kds", v2c_kds);

        // initialize the drivers
        for (int i = 0; i < motor_count; i++)
        {
            MotorType type = static_cast<MotorType>(motor_modes[i]);
            int id = motor_ids[i];
            driver_[i] = new DjiDriver(id, type);
            driver_[i]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            driver_[i]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
        }
    }

    void update_pid()
    {
        p2v_kps = this->get_parameter("kps").as_double_array();
        p2v_kis = this->get_parameter("kis").as_double_array();
        p2v_kds = this->get_parameter("kds").as_double_array();

        v2c_kps = this->get_parameter("kps").as_double_array();
        v2c_kis = this->get_parameter("kis").as_double_array();
        v2c_kds = this->get_parameter("kds").as_double_array();
        
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