#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "meta_hardware/dm_imu_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace meta_hardware
{
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    DMIMUInterface::~DMIMUInterface() {}
    hardware_interface::CallbackReturn DMIMUInterface::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DMIMUInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger("dm_imu_interface"));
        tty_devpath_ = info_.hardware_parameters.at("tty_devpath");
        sensor_name_ = info_.hardware_parameters.at("sensor_name");
        dm_imu_state_.orientation_x = NaN;
        dm_imu_state_.orientation_y = NaN;
        dm_imu_state_.orientation_z = NaN;
        dm_imu_state_.orientation_w = NaN;
        dm_imu_state_.angular_velocity_x = NaN;
        dm_imu_state_.angular_velocity_y = NaN;
        dm_imu_state_.angular_velocity_z = NaN;
        dm_imu_state_.linear_acceleration_x = NaN;
        dm_imu_state_.linear_acceleration_y = NaN;
        dm_imu_state_.linear_acceleration_z = NaN;
        init_imu_serial(tty_devpath_);

        rclcpp::sleep_for(std::chrono::milliseconds(10));
        enter_setting_mode();
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        turn_on_accel();
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        turn_on_gyro();
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        turn_on_euler();
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        turn_off_quat();
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        set_output_1000HZ();
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        save_imu_para();
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        exit_setting_mode();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        rx_thread_ = std::make_unique<std::jthread>([this](std::stop_token s) { get_imu_data_thread(s); });
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DMIMUInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(sensor_name_, "orientation.x", &dm_imu_state_.orientation_x);
        state_interfaces.emplace_back(sensor_name_, "orientation.y", &dm_imu_state_.orientation_y);
        state_interfaces.emplace_back(sensor_name_, "orientation.z", &dm_imu_state_.orientation_z);
        state_interfaces.emplace_back(sensor_name_, "orientation.w", &dm_imu_state_.orientation_w);
        state_interfaces.emplace_back(sensor_name_, "angular_velocity.x", &dm_imu_state_.angular_velocity_x);
        state_interfaces.emplace_back(sensor_name_, "angular_velocity.y", &dm_imu_state_.angular_velocity_y);
        state_interfaces.emplace_back(sensor_name_, "angular_velocity.z", &dm_imu_state_.angular_velocity_z);
        state_interfaces.emplace_back(sensor_name_, "linear_acceleration.x", &dm_imu_state_.linear_acceleration_x);
        state_interfaces.emplace_back(sensor_name_, "linear_acceleration.y", &dm_imu_state_.linear_acceleration_y);
        state_interfaces.emplace_back(sensor_name_, "linear_acceleration.z", &dm_imu_state_.linear_acceleration_z);
        return state_interfaces;
    }


    hardware_interface::CallbackReturn DMIMUInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DMIMUInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DMIMUInterface::read(const rclcpp::Time& /*time*/,
                                                         const rclcpp::Duration& /*period*/)
    {

        return hardware_interface::return_type::OK;
    }
    // from dm
    // https://gitee.com/kit-miao/dm-imu/blob/master/02.例程/ROS1-noetic例程/src/dm_imu/src/imu_driver.cpp
    void DMIMUInterface::enter_setting_mode()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x06, 0x01, 0x0D};
        dm_imu_serial_driver_->port()->send(txbuf);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    void DMIMUInterface::turn_on_accel()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x01, 0x14, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void DMIMUInterface::turn_on_gyro()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x01, 0x15, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void DMIMUInterface::turn_on_euler()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x01, 0x16, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void DMIMUInterface::turn_off_quat()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x01, 0x07, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void DMIMUInterface::set_output_1000HZ()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x02, 0x01, 0x00, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void DMIMUInterface::save_imu_para()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x03, 0x01, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void DMIMUInterface::exit_setting_mode()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x06, 0x00, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void DMIMUInterface::restart_imu()
    {
        std::vector<uint8_t> txbuf = {0xAA, 0x00, 0x00, 0x0D};
        for (int i = 0; i < 5; i++)
        {
            dm_imu_serial_driver_->port()->send(txbuf);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }
    void DMIMUInterface::init_imu_serial(std::string& dm_imu_devpath)
    {
        dm_imu_ctx_ = std::make_unique<IoContext>(2);
        auto dm_imu_serial_config_ =
            std::make_unique<SerialPortConfig>(baudrate, FlowControl::NONE, Parity::NONE, StopBits::ONE);
        dm_imu_serial_driver_ = std::make_unique<SerialDriver>(*dm_imu_ctx_);
        dm_imu_serial_driver_->init_port(dm_imu_devpath, *dm_imu_serial_config_);
    }
    void DMIMUInterface::get_imu_data_thread(std::stop_token s)
    {
        int error_num = 0;
        std::vector<uint8_t> frame_header(4);
        std::vector<uint8_t> header_left(57 - 4);
        while (!s.stop_requested())
        {
            if (!dm_imu_serial_driver_->port()->is_open())
            {
                RCLCPP_WARN(*logger_, "In get_imu_data_thread,imu serial port unopen");
            }

            dm_imu_serial_driver_->port()->receive(frame_header);
            std::memcpy(&receive_data_.FrameHeader1, frame_header.data(), 4);
            if (receive_data_.FrameHeader1 == 0x55 && receive_data_.flag1 == 0xAA && receive_data_.slave_id1 == 0x01 &&
                receive_data_.reg_acc == 0x01)
            {
                dm_imu_serial_driver_->port()->receive(header_left);
                std::memcpy(&receive_data_.accx_u32, header_left.data(), 57 - 4);

                if (Get_CRC16((uint8_t*)(&receive_data_.FrameHeader1), 16) == receive_data_.crc1)
                {
                    data_.accx = *((float*)(&receive_data_.accx_u32));
                    data_.accy = *((float*)(&receive_data_.accy_u32));
                    data_.accz = *((float*)(&receive_data_.accz_u32));
                }
                if (Get_CRC16((uint8_t*)(&receive_data_.FrameHeader2), 16) == receive_data_.crc2)
                {
                    data_.gyrox = *((float*)(&receive_data_.gyrox_u32));
                    data_.gyroy = *((float*)(&receive_data_.gyroy_u32));
                    data_.gyroz = *((float*)(&receive_data_.gyroz_u32));
                }
                if (Get_CRC16((uint8_t*)(&receive_data_.FrameHeader3), 16) == receive_data_.crc3)
                {
                    data_.roll = *((float*)(&receive_data_.roll_u32));
                    data_.pitch = *((float*)(&receive_data_.pitch_u32));
                    data_.yaw = *((float*)(&receive_data_.yaw_u32));
                }

                tf2::Quaternion tmp_q;
                tmp_q.setRPY(data_.roll / 180.0 * M_PI, data_.pitch / 180.0 * M_PI, data_.yaw / 180.0 * M_PI);


                dm_imu_state_.orientation_x = tmp_q.x();
                dm_imu_state_.orientation_y = tmp_q.y();
                dm_imu_state_.orientation_z = tmp_q.z();
                dm_imu_state_.orientation_w = tmp_q.w();
                dm_imu_state_.angular_velocity_x = data_.gyrox;
                dm_imu_state_.angular_velocity_y = data_.gyroy;
                dm_imu_state_.angular_velocity_z = data_.gyroz;
                dm_imu_state_.linear_acceleration_x = data_.accx;
                dm_imu_state_.linear_acceleration_y = data_.accy;
                dm_imu_state_.linear_acceleration_z = data_.accz;
            }
            else
            {
                error_num++;
                if (error_num > 1200)
                {
                    std::cerr << "fail to get the correct imu data,finding header 0x55" << std::endl;
                }
            }
        }
    }
} // namespace meta_hardware
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(meta_hardware::DMIMUInterface, hardware_interface::SensorInterface)
