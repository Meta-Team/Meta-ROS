#include "motor_controller/dji_motor.h"
#include "motor_controller/motor_driver.h"

#include "rclcpp/rclcpp.hpp"

#define DT CALC_FREQ / 1000 // the time interval, in seconds
#define TRY_VEL std::unordered_map<MotorType, double>{{M3508, 50.0}, {M6020, 1.0}, {M2006, 50.0}}[motor_type]

umap<int, unique_ptr<DjiMotor::CanPort>> DjiMotor::can_ports{};
umap<int, std::thread> DjiMotor::rx_threads{};
umap<int, std::thread> DjiMotor::tx_threads{};
umap<int, umap<int, DjiMotor*>> DjiMotor::instances{};

DjiMotor::DjiMotor(const string& rid, const int hid, string type, string port, int cali) :
    MotorDriver(rid, hid),
    p2v_prm(0.0, 0.0, 0.0),
    v2t_prm(0.0, 0.0, 0.0)
{
    if (type == "3508") motor_type = M3508;
    else if (type == "6020") motor_type = M6020;
    else if (type == "2006") motor_type = M2006;
    else RCLCPP_ERROR(rclcpp::get_logger("dji_driver"), "Unknown motor type: %s", type.c_str());
    
    this->p2v_out = PidOutput();
    this->v2t_out = PidOutput();

    // parse the port number
    this->port = port.back() - '0';
    int fb_id = calc_fb_id();
    instances[this->port][fb_id] = this; // add the instance to the map

    create_port(this->port);
    // create the port after the instance is added
    // in case the threads race to access the instance

    calc_thread = std::thread(&DjiMotor::calc_loop, this);
    cali_thread = std::thread(&DjiMotor::cali_loop, this, cali);
}

DjiMotor::~DjiMotor()
{
    if (calc_thread.joinable()) calc_thread.join();
    if (cali_thread.joinable()) cali_thread.join();
    destroy_port(port);
}

void DjiMotor::set_goal(double goal_pos, double goal_vel, double goal_tor)
{
    if (!ready) return;
    this->goal_pos = goal_pos + zero;
    this->goal_vel = goal_vel;
    this->torque = goal_tor;
}

void DjiMotor::set_param(double p2v_kp, double p2v_ki, double p2v_kd,
    double v2t_kp, double v2t_ki, double v2t_kd)
{
    p2v_prm.kp = p2v_kp;
    p2v_prm.ki = p2v_ki;
    p2v_prm.kd = p2v_kd;

    v2t_prm.kp = v2t_kp;
    v2t_prm.ki = v2t_ki;
    v2t_prm.kd = v2t_kd;
}

std::tuple<double, double, double> DjiMotor::get_state()
{
    return std::make_tuple(
        present_data.position - zero,
        present_data.velocity,
        present_data.torque
    );
}

void DjiMotor::print_info()
{
    RCLCPP_INFO(rclcpp::get_logger("dji_motor"),
        "DjiMotor rid %s hid %d created: p2v (%f, %f, %f), v2t (%f, %f, %f)",
        rid.c_str(), hid,
        p2v_prm.kp, p2v_prm.ki, p2v_prm.kd,
        v2t_prm.kp, v2t_prm.ki, v2t_prm.kd
    );
}

void DjiMotor::vel2torque()
{
    double prev_error = vel_error;
    vel_error = goal_vel - present_data.velocity;

    v2t_out.p = v2t_prm.kp * vel_error;
    v2t_out.i += v2t_prm.ki * vel_error * DT; curb(v2t_out.i, static_cast<double>(T_MAX) / 4);
    v2t_out.d = v2t_prm.kd * (vel_error - prev_error) / DT;

    this->torque = v2t_out.sum();

    // restrict the torque
    curb(torque, T_MAX);
}

void DjiMotor::pos2velocity()
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

int DjiMotor::calc_fb_id()
{
    switch (motor_type)
    {
    case M3508:
        return 0x200 + hid;
    case M6020:
        return 0x204 + hid;
    case M2006:
        return 0x200 + hid;
    default:
        return 0;
    }

}

void DjiMotor::create_port(int port)
{
    if (can_ports.find(port) == can_ports.end())
    {
        can_ports[port] = std::make_unique<CanPort>(port);
        rx_threads[port] = std::thread(&DjiMotor::rx_loop, port);
        tx_threads[port] = std::thread(&DjiMotor::tx_loop, port);
    }
}

void DjiMotor::destroy_port(int port)
{
    if (instances[port].empty())
    {
        can_ports.erase(port);
        if (rx_threads[port].joinable()) rx_threads[port].join();
        if (tx_threads[port].joinable()) tx_threads[port].join();
    }
}

void DjiMotor::rx_loop(int port)
{
    auto& can_port = can_ports[port];
    auto& instances = DjiMotor::instances[port]; // instances on this port

    while (rclcpp::ok())
    {
        can_port->rx();
        int fb_id = can_port->rx_frame.can_id;

        // find corresponding instance and let it process
        if (instances.find(fb_id) != instances.end())
            instances[fb_id]->process_rx();
    }
}

void DjiMotor::tx_loop(int port)
{
    auto& can_port = can_ports[port];
    while (rclcpp::ok())
    {
        // calculate in another thread
        can_port->tx();
        rclcpp::sleep_for(std::chrono::milliseconds(TX_FREQ));
    }
}

void DjiMotor::cali_loop(int dir)
{
    auto log = rclcpp::get_logger("dji_driver");

#define NOW rclcpp::Clock().now().seconds()
    const auto start = NOW;

    if (dir == 0)
    {
        zero = 0.0;
        // delay for a while
        ready = false;
        while (NOW - start < CALI_TIMEOUT && rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(CALC_FREQ));
        }
        ready = true;
        return;
    }

    // if dir == 1 or -1, try to find zero
    bool found = false;
    auto last_not_jammed_moment = NOW;

    while (NOW - start < CALI_TIMEOUT)
    {
        // set goal and get feedback
        goal_pos = NaN;
        goal_vel = dir * TRY_VEL;
        auto fb_vel = present_data.velocity;

        // check if the motor is jammed
        if (std::abs(fb_vel) > TRY_VEL / 5) last_not_jammed_moment = NOW;

        // if jammed long enough
        if (NOW - last_not_jammed_moment > JAMMED_THRESHOLD)
        {
            found = true;
            break;
        }

        // sleep
        rclcpp::sleep_for(std::chrono::milliseconds(CALC_FREQ));
    }

    ready = true;
    if (found)
    {
        zero = present_data.position;
        set_goal(0.0, NaN, NaN); // keep the motor still at zero
        RCLCPP_INFO(log, "Motor %s zero found: %f", rid.c_str(), zero);
    } else {
        zero = 0.0;
        set_goal(NaN, 0.0, NaN); // keep the motor still
        RCLCPP_WARN(log, "Motor %s zero not found", rid.c_str());
    }

    // delay for a while
    ready = false;
    while (NOW - start < CALI_TIMEOUT && rclcpp::ok())
    {
        rclcpp::sleep_for(std::chrono::milliseconds(CALC_FREQ));
    }
    ready = true;

#undef NOW
}

void DjiMotor::calc_loop()
{
    while (rclcpp::ok())
    {
        calc_tx();
        rclcpp::sleep_for(std::chrono::milliseconds(CALC_FREQ));
    }

}

void DjiMotor::calc_tx() // calc and write
{
    if (!std::isnan(goal_pos)) pos2velocity(); // this would overwrite goal_vel
    if (!std::isnan(goal_vel)) vel2torque(); // this would overwrite torque
    // if both are NaN, then use the torque value directly

    auto& tx_frame_200 = can_ports[port]->tx_frame_200;
    auto& tx_frame_1ff = can_ports[port]->tx_frame_1ff;
    auto& tx_frame_2ff = can_ports[port]->tx_frame_2ff;

    // write the data to the frame
    switch (motor_type)
    {
    case M3508:
    {
        int16_t torque_data = static_cast<int16_t>(torque / T_MAX * 16384);
        if (hid <= 4)
        {
            tx_frame_200.data[2 * hid - 2] = (uint8_t)(torque_data >> 8);
            tx_frame_200.data[2 * hid - 1] = (uint8_t)(torque_data & 0xff);
        } else {
            tx_frame_1ff.data[2 * (hid - 4) - 2] = (uint8_t)(torque_data >> 8);
            tx_frame_1ff.data[2 * (hid - 4) - 1] = (uint8_t)(torque_data & 0xff);
        }
        break;
    }
    case M6020:
    {
        int16_t torque_data = static_cast<int16_t>(torque / T_MAX * 30000);
        if (hid <= 4)
        {
            tx_frame_1ff.data[2 * hid - 2] = (uint8_t)(torque_data >> 8);
            tx_frame_1ff.data[2 * hid - 1] = (uint8_t)(torque_data & 0xff);
        } else {
            tx_frame_2ff.data[2 * (hid - 4) - 2] = (uint8_t)(torque_data >> 8);
            tx_frame_2ff.data[2 * (hid - 4) - 1] = (uint8_t)(torque_data & 0xff);
        }
        break;
    }
    case M2006:
    {
        int16_t torque_data = static_cast<int16_t>(torque / T_MAX * 16384);
        if (hid <= 4)
        {
            tx_frame_200.data[2 * hid - 2] = (uint8_t)(torque_data >> 8);
            tx_frame_200.data[2 * hid - 1] = (uint8_t)(torque_data & 0xff);
        } else {
            tx_frame_1ff.data[2 * (hid - 4) - 2] = (uint8_t)(torque_data >> 8);
            tx_frame_1ff.data[2 * (hid - 4) - 1] = (uint8_t)(torque_data & 0xff);
        }
        break;
    }
    default:
        break;
    }
}

void DjiMotor::process_rx()
{
    auto& rx_frame = can_ports[port]->rx_frame;

    // parse the frame
    int16_t pos_raw = (static_cast<int16_t>(rx_frame.data[0]) << 8) | (rx_frame.data[1] & 0xFF);
    int16_t vel_raw = (static_cast<int16_t>(rx_frame.data[2]) << 8) | (rx_frame.data[3] & 0xFF);
    int16_t tor_raw = (static_cast<int16_t>(rx_frame.data[4]) << 8) | (rx_frame.data[5] & 0xFF);
    
    present_data.update_pos((double)pos_raw / 8192.0f * 2 * 3.1415926f); // rad
    present_data.velocity = (double)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s
    
    switch (motor_type)
    {
    case M3508:
    case M2006:
        present_data.torque = (double)tor_raw / 16384 * 20; // actually torque, Ampere
        break;
    case M6020:
        present_data.torque = (double)tor_raw / 30000 * 24; // actually torque, Ampere
        break;
    }
}

void DjiMotor::curb(double &val, double limit)
{
    if (val > limit) val = limit;
    else if (val < -limit) val = -limit;
}