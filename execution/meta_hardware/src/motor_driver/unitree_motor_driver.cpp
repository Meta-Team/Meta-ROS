#include <cmath>
#include <stdexcept>
#include <string>

#include "meta_hardware/motor_driver/unitree_motor_driver.hpp"

namespace meta_hardware {
constexpr float NaN_f = std::numeric_limits<float>::quiet_NaN();
UnitreeMotor::UnitreeMotor(const std::unordered_map<std::string, std::string> &motor_param) {
    
    // Parse ID
    if (motor_param.find("motor_id") != motor_param.end()) {
        motor_id_ = static_cast<uint8_t>(std::stoi(motor_param.at("motor_id")));
    } else {
         // Fallback or error if ID is mandatory
         motor_id_ = 0; 
    }

    // Parse PD gains
    if (motor_param.find("kp") != motor_param.end()) {
        kp_ = std::stod(motor_param.at("kp"));
    }
    if (motor_param.find("kd") != motor_param.end()) {
        kd_ = std::stod(motor_param.at("kd"));
    }
    
    // Hardcoded Limits/Defaults
    // Note: queryGearRatio is an SDK function. 
    // We assume MotorType::GO_M8010_6 based on your requirements.
    gear_ratio_ = queryGearRatio(MotorType::GO_M8010_6); 
}

void UnitreeMotor::initialize_cmd(MotorCmd& cmd, MotorData& data) {
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = motor_id_;
    cmd.kp = 0.0;
    cmd.kd = 0.0;
    cmd.q = 0.0;
    cmd.dq = 0.0;
    cmd.tau = 0.0;

    data.motorType = MotorType::GO_M8010_6;
    data.q = NaN_f;
    data.dq = NaN_f;
    data.tau = NaN_f;
    data.temp = NaN_f;
}

void UnitreeMotor::set_motor_command(MotorCmd& cmd, double position, double velocity, double effort) {
    // Ensure ID and Type persist
    cmd.id = motor_id_;
    cmd.motorType = MotorType::GO_M8010_6;

    // Filter NaNs for safety
    if (std::isnan(position) && std::isnan(velocity) && std::isnan(effort)){
        // no force applied, but will receive state feedback
        // used when boot up and no command is valid, you can still get state of motors
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    } else{
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        if (std::isnan(position)) position = 0.0; // Or handle as "keep previous" in driver
        if (std::isnan(velocity)) velocity = 0.0;
        if (std::isnan(effort)) effort = 0.0;
    }

    // Apply PD gains from config
    cmd.kp = kp_;
    cmd.kd = kd_;

    // Unitree Go1 Motors usually expect Position/Velocity at the Output Shaft (Radians)
    // The internal controller handles the gear ratio for q and dq if configured correctly,
    // BUT usually with this SDK, you send values multiplied by gear ratio if driving the rotor directly,
    // or output shaft values if the internal mode handles it.
    // Based on the old file `dq = velocity * queryGearRatio(...)`, it suggests we need to manually apply gear ratio?
    // However, usually FOC mode in Unitree SDK expects physical units (radians, rad/s).
    // Let's trust your previous code snippet: `dq = velocity * queryGearRatio`.
    // If that is true, then `q` should likely also be `position * gear_ratio`.
    
    // Replicating logic from your snippet:
    cmd.q = position * gear_ratio_;
    cmd.dq = velocity * gear_ratio_; 
    
    // Torque is usually pre-reduction for the motor core? Or post-reduction?
    // Standard relation: Tau_motor = Tau_output / GearRatio.
    cmd.tau = effort / gear_ratio_;
}

std::tuple<double, double, double> UnitreeMotor::get_motor_feedback(const MotorData& data) const {
    // Reverse the gear ratio logic
    double position = data.q / gear_ratio_;
    double velocity = data.dq / gear_ratio_;
    double effort = data.tau * gear_ratio_; // Torque at output shaft

    return {position, velocity, effort};
}

} // namespace meta_hardware