#include "shoot_controller/shoot_controller.hpp"
#include <controller_interface/controller_interface_base.hpp>


using hardware_interface::HW_IF_VELOCITY;

namespace meta_shoot_controller {
    ShootController::ShootController() {
        // Empty constructor
    }


    controller_interface::CallbackReturn  
    ShootController::on_init() {
        // Empty on_init
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn 
    ShootController::on_configure(const rclcpp_lifecycle::State &previous_state){
        // Empty on_configure
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn 
    ShootController::on_activate(const rclcpp_lifecycle::State &previous_state){
        // Empty on_activate
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    ShootController::on_deactivate(const rclcpp_lifecycle::State &previous_state){
        // Empty on_deactivate
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // controller_interface::InterfaceConfiguration 
    // ShootController::command_interface_configuration() const{
    //         // return controller_interface::InterfaceConfiguration();
    // }

}