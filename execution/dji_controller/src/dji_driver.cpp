#include "dji_controller/dji_driver.h"
#include "dji_controller/can_driver.hpp"
#include "dji_controller/can_port.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <linux/can.h>
#include <memory>
#include <queue>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tuple>

#define DT CALC_FREQ / 1000 // the time interval, in seconds

// static members
umap<int, unique_ptr<CanPort>> DjiDriver::can_ports{};
umap<int, std::thread> DjiDriver::rx_threads{};
umap<int, std::thread> DjiDriver::tx_threads{};
vector<std::shared_ptr<DjiDriver>> DjiDriver::instances{};

DjiDriver::DjiDriver(const string& rid, const int hid, string type, string can_port) :
    p2v_prm(0.1, 0.01, 0.1),
    v2c_prm(0.004, 0.00003, 0.1),
    hid(hid),
    rid(rid)
{
    if (type == "3508") motor_type = M3508;
    else if (type == "6020") motor_type = M6020;
    else if (type == "2006") motor_type = M2006;
    else std::cerr << "Unknown motor type: " << type << std::endl;
    
    this->p2v_out = PidOutput();
    this->v2c_out = PidOutput();

    instances.push_back(std::shared_ptr<DjiDriver>(this));
    set_port(can_port.back() - '0');

    last_command = 0.0;
    timeout_thread = std::thread(&DjiDriver::check_timeout, this);
    calc_thread = std::thread(&DjiDriver::calc_loop, this);
}

DjiDriver::~DjiDriver()
{
    if (timeout_thread.joinable()) timeout_thread.join();
}

void DjiDriver::check_timeout()
{
    rclcpp::sleep_for(std::chrono::seconds(1));
    while (rclcpp::ok())
    {
        if (rclcpp::Clock().now().seconds() - last_command > 1.0)
        {
            current = 0.0;
            RCLCPP_INFO(rclcpp::get_logger("dji_driver"), "Motor %s timeout, stop the motor", rid.c_str());
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

void DjiDriver::set_port(int port)
{
    this->port = port;
    if (can_ports.find(port) == can_ports.end())
    {
        can_ports[port] = std::make_unique<CanPort>(port);
        rx_threads[port] = std::thread(&DjiDriver::rx_loop, port);
        tx_threads[port] = std::thread(&DjiDriver::tx_loop, port);
    }
}

void DjiDriver::rx_loop(int port)
{
    auto& can_port = can_ports[port];
    while (rclcpp::ok())
    {
        can_port->rx();
        for (auto& instance : instances)
            instance->process_rx();
    }
}

void DjiDriver::tx_loop(int port)
{
    auto& can_port = can_ports[port];
    while (rclcpp::ok())
    {
        // calculate in another thread
        can_port->tx();
        rclcpp::sleep_for(std::chrono::milliseconds(TX_FREQ));
    }
}

void DjiDriver::calc_loop()
{
    while (rclcpp::ok())
    {
        calc_tx();
        rclcpp::sleep_for(std::chrono::milliseconds(CALC_FREQ));
    }

}

void DjiDriver::set_goal(double goal_pos, double goal_vel, double goal_cur)
{
    last_command = rclcpp::Clock().now().seconds();
    this->goal_pos = goal_pos;
    this->goal_vel = goal_vel;
    this->current = goal_cur;
}

void DjiDriver::set_p2v_pid(double kp, double ki, double kd)
{
    p2v_prm.kp = kp;
    p2v_prm.ki = ki;
    p2v_prm.kd = kd;
}

void DjiDriver::set_v2c_pid(double kp, double ki, double kd)
{
    v2c_prm.kp = kp;
    v2c_prm.ki = ki;
    v2c_prm.kd = kd;
}

void DjiDriver::vel2current()
{
    double prev_error = vel_error;
    vel_error = goal_vel - present_data.velocity;

    v2c_out.p = v2c_prm.kp * vel_error;
    v2c_out.i += v2c_prm.ki * vel_error * DT; curb(v2c_out.i, static_cast<double>(I_MAX) / 4);
    v2c_out.d = v2c_prm.kd * (vel_error - prev_error) / DT;

    this->current = v2c_out.sum();

    // restrict the current
    curb(current, I_MAX);
}

void DjiDriver::pos2velocity()
{
    double prev_error = pos_error;
    pos_error = goal_pos - present_data.position;

    p2v_out.p = p2v_prm.kp * pos_error;
    p2v_out.i += p2v_prm.ki * pos_error * DT; curb(p2v_out.i, static_cast<double>(V_MAX) / 4);
    p2v_out.d = p2v_prm.kd * (pos_error - prev_error) / DT;

    this->goal_vel = p2v_out.sum();

    // restrict the velocity
    curb(goal_vel, V_MAX);
}

void DjiDriver::process_rx()
{
    auto& rx_frame = can_ports[port]->rx_frame;

    // check if the frame is for this motor
    switch (motor_type)
    {
    case M3508:
        if ((int)rx_frame.can_id != 0x200 + hid) return;
        break;
    case M6020:
        if ((int)rx_frame.can_id != 0x204 + hid) return;
        break;
    case M2006:
        if ((int)rx_frame.can_id != 0x200 + hid) return;
        break;
    default:
        return;
    }

    // parse the frame
    int16_t pos_raw = (static_cast<int16_t>(rx_frame.data[0]) << 8) | (rx_frame.data[1] & 0xFF);
    int16_t vel_raw = (static_cast<int16_t>(rx_frame.data[2]) << 8) | (rx_frame.data[3] & 0xFF);
    int16_t tor_raw = (static_cast<int16_t>(rx_frame.data[4]) << 8) | (rx_frame.data[5] & 0xFF);
    
    present_data.update_pos((double)pos_raw / 8192.0f * 2 * 3.1415926f); // rad
    present_data.velocity = (double)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s
    present_data.torque = (double)tor_raw / 16384 * 20; // actually current, Ampere
}

void DjiDriver::calc_tx() // calc and write
{
    if (!std::isnan(goal_pos)) pos2velocity(); // this would overwrite goal_vel
    if (!std::isnan(goal_vel)) vel2current(); // this would overwrite current
    // if both are NaN, then use the current value directly

    auto& tx_frame_200 = can_ports[port]->tx_frame_200;
    auto& tx_frame_1ff = can_ports[port]->tx_frame_1ff;
    auto& tx_frame_2ff = can_ports[port]->tx_frame_2ff;

    // write the data to the frame
    switch (motor_type)
    {
    case M3508:
    {
        int16_t current_data = static_cast<int16_t>(current / I_MAX * 16384);
        if (hid <= 4)
        {
            tx_frame_200.data[2 * hid - 2] = (uint8_t)(current_data >> 8);
            tx_frame_200.data[2 * hid - 1] = (uint8_t)(current_data & 0xff);
        } else {
            tx_frame_1ff.data[2 * (hid - 4) - 2] = (uint8_t)(current_data >> 8);
            tx_frame_1ff.data[2 * (hid - 4) - 1] = (uint8_t)(current_data & 0xff);
        }
        break;
    }
    case M6020:
    {
        int16_t current_data = static_cast<int16_t>(current / I_MAX * 30000);
        if (hid <= 4)
        {
            tx_frame_1ff.data[2 * hid - 2] = (uint8_t)(current_data >> 8);
            tx_frame_1ff.data[2 * hid - 1] = (uint8_t)(current_data & 0xff);
        } else {
            tx_frame_2ff.data[2 * (hid - 4) - 2] = (uint8_t)(current_data >> 8);
            tx_frame_2ff.data[2 * (hid - 4) - 1] = (uint8_t)(current_data & 0xff);
        }
        break;
    }
    case M2006:
    {
        int16_t current_data = static_cast<int16_t>(current / I_MAX * 16384);
        if (hid <= 4)
        {
            tx_frame_200.data[2 * hid - 2] = (uint8_t)(current_data >> 8);
            tx_frame_200.data[2 * hid - 1] = (uint8_t)(current_data & 0xff);
        } else {
            tx_frame_1ff.data[2 * (hid - 4) - 2] = (uint8_t)(current_data >> 8);
            tx_frame_1ff.data[2 * (hid - 4) - 1] = (uint8_t)(current_data & 0xff);
        }
        break;
    }
    default:
        break;
    }
}

std::tuple<double, double, double> DjiDriver::get_state()
{
    return std::make_tuple(
        present_data.position,
        present_data.velocity,
        present_data.torque
    );
}

void DjiDriver::curb(double &val, double limit)
{
    if (val > limit) val = limit;
    else if (val < -limit) val = -limit;
}