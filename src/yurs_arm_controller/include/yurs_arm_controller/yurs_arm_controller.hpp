#ifndef YURS_ARM_CONTROLLER__YURS_ARM_CONTROLLER_HPP_
#define YURS_ARM_CONTROLLER__YURS_ARM_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "visibility_control.h"

namespace yurs_arm_controller
{
  class YursArmController : public controller_interface::ControllerInterface
  {
  public:
    YURS_ARM_CONTROLLER_PUBLIC
    YursArmController();

    YURS_ARM_CONTROLLER_PUBLIC
    controller_interface::return_type init(const std::string& controller_name,
                                          const hardware_interface::HWResourceManager& resources) override;

    YURS_ARM_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfigurationSharedPtr
    command_interface_configuration() const override;

    YURS_ARM_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfigurationSharedPtr
    state_interface_configuration() const override;

    YURS_ARM_CONTROLLER_PUBLIC
    controller_interface::return_type on_configure(const controller_interface::ControllerConfiguration& config,
                                                   const hardware_interface::HWResourceManager& resources) override;

    YURS_ARM_CONTROLLER_PUBLIC
    controller_interface::return_type on_activate() override;

    YURS_ARM_CONTROLLER_PUBLIC
    controller_interface::return_type on_deactivate() override;

    YURS_ARM_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  protected:
    // Optional: Add protected string vectors for joint and interface names.
    std::vector<std::string> joint_names_;
    std::vector<std::string> interface_names_;
  };

}  // namespace yurs_arm_controller

#endif  // YURS_ARM_CONTROLLER__YURS_ARM_CONTROLLER_HPP_
