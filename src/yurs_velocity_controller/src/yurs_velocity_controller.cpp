#include "yurs_velocity_controller/yurs_velocity_controller.hpp"

namespace yurs_velocity_controller
{

    Yurs_velocity_controller::Yurs_velocity_controller(/* args */)
    {
    }

    Yurs_velocity_controller::~Yurs_velocity_controller()
    {
    }

    controller_interface::CallbackReturn
    Yurs_velocity_controller::on_init() override
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    Yurs_velocity_controller::command_interface_configuration() const override
    {
    }

    controller_interface::InterfaceConfiguration
    Yurs_velocity_controller::state_interface_configuration() const override
    {
    }

    controller_interface::CallbackReturn
    Yurs_velocity_controller::on_configure(const rclcpp_lifecycle::State &previous_state) override
    {
    }

    controller_interface::CallbackReturn
    Yurs_velocity_controller::on_activate(const rclcpp_lifecycle::State &previous_state) override
    {
    }

    controller_interface::CallbackReturn
    Yurs_velocity_controller::on_deactivate(const rclcpp_lifecycle::State &previous_state) override
    {
    }

    controller_interface::return_type
    Yurs_velocity_controller::update(const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
    }

}
