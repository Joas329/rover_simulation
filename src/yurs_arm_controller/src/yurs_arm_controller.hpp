#ifndef YURS_ARM_CONTROLLER
#define YURS_ARM_CONTROLLER
#include "yurs_hardware/yurs_motor.hpp"

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"


#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
namespace yurs_arm_controller
{
    class YURS_Arm_Controller : public controller_interface::ControllerInterface
    {
        public:
            YURS_Arm_Controller();

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::return_type update(
                const rclpp::Time & time, const rclpp::Duration & period) override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_error(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_shutdown(
                const rclcpp_lifecycle::State & previous_state) override;


    };
}

#endif