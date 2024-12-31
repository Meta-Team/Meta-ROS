#include "shoot_controller/shoot_controller.hpp"
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>


using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();



namespace shoot_controller {
    
    controller_interface::CallbackReturn  
    ShootController::on_init() {
        friction_wheel_on_ = false;
        bullet_loader_on_ = false;
        try {
            param_listener_ = std::make_shared<shoot_controller::ParamListener>(get_node());
        } catch (const std::exception &e) {
            std::cerr << "Exception thrown during controller's init with message: "
                    << e.what() << std::endl;
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn 
    ShootController::on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
        params_ = param_listener_->get_params();
        friction_wheel_speed_ = params_.friction_wheel.velocity;

        fric1_vel_pid_ = std::make_shared<control_toolbox::PidROS>(
            get_node(), "gains.friction_wheel1_joint_vel2eff", true
        );
        fric2_vel_pid_ = std::make_shared<control_toolbox::PidROS>(
            get_node(), "gains.friction_wheel2_joint_vel2eff", true
        );
        load_vel_pid_ = std::make_shared<control_toolbox::PidROS>(
            get_node(), "gains.bullet_loader_joint_vel2eff", true
        );

        if (!fric1_vel_pid_->initPid()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for friction wheel 1");
            return controller_interface::CallbackReturn::FAILURE;
        }
        if (!fric2_vel_pid_->initPid()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for friction wheel 2");
            return controller_interface::CallbackReturn::FAILURE;
        }
        if (!load_vel_pid_->initPid()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for bullet load");
            return controller_interface::CallbackReturn::FAILURE;
        }

        // topics QoS
        auto subscribers_qos = rclcpp::SystemDefaultsQoS();
        subscribers_qos.keep_last(1);
        subscribers_qos.best_effort();

        // Reference Subscriber
        ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
            "~/reference", subscribers_qos,
            std::bind(&ShootController::reference_callback, this, std::placeholders::_1));

        
        auto msg = std::make_shared<ControllerReferenceMsg>();
        reset_controller_reference_msg(msg, get_node());
        input_ref_.writeFromNonRT(msg);

        try {
            // State publisher
            s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
                "~/controller_state", rclcpp::SystemDefaultsQoS());
            state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
        } catch (const std::exception &e) {
            std::cerr << "Exception thrown during publisher creation at configure "
                        "stage with message: "
                    << e.what() << std::endl;
            return controller_interface::CallbackReturn::ERROR;
        }

        state_publisher_->lock();
        state_publisher_->msg_.dof_states.resize(3);
        state_publisher_->msg_.dof_states[0].name = "friction1_vel";
        state_publisher_->msg_.dof_states[1].name = "friction2_vel";
        state_publisher_->msg_.dof_states[2].name = "load_vel"; 
        state_publisher_->unlock();

        RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn 
    ShootController::on_activate(const rclcpp_lifecycle::State &/*previous_state*/){
        reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
        
        reference_interfaces_.assign(reference_interfaces_.size(), NaN);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    ShootController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/){
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration 
    ShootController::command_interface_configuration() const{
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

        command_interfaces_config.names.reserve(3);
        command_interfaces_config.names.push_back("fric1_shooter_joint/" + std::string(HW_IF_EFFORT));
        command_interfaces_config.names.push_back("fric2_shooter_joint/" + std::string(HW_IF_EFFORT));
        command_interfaces_config.names.push_back( "loader_shooter_joint/" + std::string(HW_IF_EFFORT));

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration 
        ShootController::state_interface_configuration() const{
                
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type =
            controller_interface::interface_configuration_type::INDIVIDUAL;

        // Joint position state of yaw gimbal is required
        state_interfaces_config.names.reserve(3);
        state_interfaces_config.names.push_back("fric1_shooter_joint/" + std::string(HW_IF_VELOCITY));
        state_interfaces_config.names.push_back("fric2_shooter_joint/" + std::string(HW_IF_VELOCITY));
        state_interfaces_config.names.push_back("loader_shooter_joint/" + std::string(HW_IF_VELOCITY));

    return state_interfaces_config;
    }

    void ShootController::reset_controller_reference_msg(
        const std::shared_ptr<ControllerReferenceMsg> &msg,
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & /*node*/) {
        msg->fric_state = false;
        msg->fric_state = false;
        msg->feed_speed = NaN;
    }

    controller_interface::return_type ShootController::update_reference_from_subscribers(){
        auto current_ref = *(input_ref_.readFromRT()); // A shared_ptr must be allocated
                                                   // immediately to prevent dangling
        if (!std::isnan(current_ref->feed_speed)) {
            if(current_ref->fric_state){
                reference_interfaces_[0] = friction_wheel_speed_;
                friction_wheel_on_ = true;
            }else{
                reference_interfaces_[0] = 0.0;
                friction_wheel_on_ = false;
            }

            if(current_ref->feed_state){
                reference_interfaces_[1] = current_ref->feed_speed;
                bullet_loader_on_ = true;
            }else{
                reference_interfaces_[1] = 0.0;
                bullet_loader_on_ = false;
            }
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type
    ShootController::update_and_write_commands(const rclcpp::Time &  time ,
                                const rclcpp::Duration &  period ){

        double fric1_vel_ref = NaN, fric2_vel_ref = NaN, load_vel_ref = NaN;
        double fric1_vel_fb = NaN, fric2_vel_fb = NaN, load_vel_fb = NaN;
        double fric1_vel_err = NaN, fric2_vel_err = NaN, load_vel_err = NaN;
        double fric1_eff_cmd = NaN, fric2_eff_cmd = NaN, load_eff_cmd = NaN;


        fric1_vel_ref = reference_interfaces_[0];
        fric2_vel_ref = reference_interfaces_[0];
        load_vel_ref = reference_interfaces_[1];

        fric1_vel_fb = state_interfaces_[0].get_value();
        fric2_vel_fb = state_interfaces_[1].get_value();
        load_vel_fb = state_interfaces_[2].get_value();

        if(!std::isnan(fric1_vel_ref) && !std::isnan(fric1_vel_fb) &&
            !std::isnan(fric2_vel_ref) && !std::isnan(fric2_vel_fb) &&
            !std::isnan(load_vel_ref) && !std::isnan(load_vel_fb)){
                
            fric1_vel_err = fric1_vel_ref - fric1_vel_fb;
            fric2_vel_err = fric2_vel_ref - fric2_vel_fb;
            load_vel_err = load_vel_ref - load_vel_fb;

            fric1_eff_cmd = fric1_vel_pid_->computeCommand(fric1_vel_err, period);
            fric2_eff_cmd = fric2_vel_pid_->computeCommand(fric2_vel_err, period);
            load_eff_cmd = load_vel_pid_->computeCommand(load_vel_err, period);

            command_interfaces_[0].set_value(fric1_eff_cmd);
            command_interfaces_[1].set_value(fric2_eff_cmd);
            command_interfaces_[2].set_value(load_eff_cmd);
        }

        
        
        // Publish state
    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.header.stamp = time;

        state_publisher_->msg_.dof_states[0].reference = fric1_vel_ref;
        state_publisher_->msg_.dof_states[0].feedback = fric1_vel_fb;
        state_publisher_->msg_.dof_states[0].error = fric1_vel_err;
        state_publisher_->msg_.dof_states[0].time_step = period.seconds();
        state_publisher_->msg_.dof_states[0].output = fric1_eff_cmd;

        state_publisher_->msg_.dof_states[1].reference = fric2_vel_ref;
        state_publisher_->msg_.dof_states[1].feedback = fric2_vel_fb;
        state_publisher_->msg_.dof_states[1].error = fric2_vel_err;
        state_publisher_->msg_.dof_states[1].time_step = period.seconds();
        state_publisher_->msg_.dof_states[1].output = fric2_eff_cmd;
        
        state_publisher_->msg_.dof_states[2].reference = load_vel_ref;
        state_publisher_->msg_.dof_states[2].feedback = load_vel_fb;
        state_publisher_->msg_.dof_states[2].error = load_vel_err;
        state_publisher_->msg_.dof_states[2].time_step = period.seconds();
        state_publisher_->msg_.dof_states[2].output = load_eff_cmd;

        state_publisher_->unlockAndPublish();
    }
        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> 
    ShootController::on_export_reference_interfaces() {
        reference_interfaces_.resize(2, NaN);

        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.reserve(reference_interfaces_.size());

        reference_interfaces.emplace_back(get_node()->get_name(),
                                        std::string("friction_wheel/") + HW_IF_EFFORT,
                                        &reference_interfaces_[0]);

        reference_interfaces.emplace_back(get_node()->get_name(),
                                        std::string("bullet_loader/") + HW_IF_EFFORT,
                                        &reference_interfaces_[1]);
        return reference_interfaces;
    }

    void ShootController::reference_callback(
        const std::shared_ptr<ControllerReferenceMsg> msg) {
        input_ref_.writeFromNonRT(msg);
    }   

    bool ShootController::on_set_chained_mode(bool chained_mode){
        return true || chained_mode;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(shoot_controller::ShootController,
                       controller_interface::ChainableControllerInterface)