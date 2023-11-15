#ifndef _YURS_VELOCITY_CONTROLLER_HPP_
#define _YURS_VELOCITY_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
// #include "yurs_velocity_controller/visibility_control.h"
#include "visibility_control.h" // testing

namespace yurs_velocity_controller
{
    class Yurs_velocity_controller : public controller_interface::ControllerInterface
    {
    private:
        /* data */

    protected:
        // Optional: Add protected string vectors for joint and interface names.
        std::vector<std::string> joint_names_;
        std::vector<std::string> interface_names_;

    public:
        YURS_VELOCITY_CONTROLLERS_PUBLIC
        Yurs_velocity_controller(/* args */);

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        ~Yurs_velocity_controller();

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        YURS_VELOCITY_CONTROLLERS_PUBLIC
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };

} // namespace yurs_velocity_controller

#endif // _YURS_VELOCITY_CONTROLLER_HPP_