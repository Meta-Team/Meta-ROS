#ifndef META_HARDWARE__MOTOR_NETWORK__UNITREE_MOTOR_NETWORK_HPP_
#define META_HARDWARE__MOTOR_NETWORK__UNITREE_MOTOR_NETWORK_HPP_

#include <vector>
#include <string>
#include <memory>
#include <tuple>
#include <unordered_map>

#include "unitree_sdk/unitreeMotor.h"
#include "unitree_sdk/SerialPort.h"

#include "meta_hardware/motor_driver/unitree_motor_driver.hpp"

namespace meta_hardware {

class UnitreeMotorNetwork {
  public:
    UnitreeMotorNetwork(const std::string &tty_devpath,
                        const std::vector<std::unordered_map<std::string, std::string>> &joint_params);
    ~UnitreeMotorNetwork();

    /**
     * @brief Performs the blocking I/O transaction.
     * Sends the current send_cmds_ vector and populates recv_datas_.
     */
    void sync_read_write();

    /**
     * @brief Extract state from the received data buffer
     */
    std::tuple<double, double, double> read_state(uint32_t joint_index) const;

    /**
     * @brief Update the command buffer for the next sync
     */
    void write_command(uint32_t joint_index, double position, double velocity, double effort);

  private:
    std::unique_ptr<SerialPort> serial_port_;

    // Bulk communication vectors required by Unitree SDK
    std::vector<MotorCmd> send_cmds_;
    std::vector<MotorData> recv_datas_;

    // Drivers to handle logic per joint
    std::vector<std::shared_ptr<UnitreeMotor>> unitree_motors_;
};

} // namespace meta_hardware

#endif // META_HARDWARE__MOTOR_NETWORK__UNITREE_MOTOR_NETWORK_HPP_