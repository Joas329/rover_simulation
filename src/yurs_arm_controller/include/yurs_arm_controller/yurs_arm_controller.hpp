#ifndef YURS_ARM_CONTROLLER__YURS_ARM_CONTROLLER_HPP_
#define YURS_ARM_CONTROLLER__YURS_ARM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "yurs_arm_controller/yurs_arm_controllers_base.hpp"
#include "yurs_arm_controller/visibility_control.h"

namespace yurs_arm_controller
{
  class YursArmController : public YursArmControllersBase
  {
  public:
    YURS_ARM_CONTROLLER_PUBLIC
    YursArmController();

  protected:
    void declare_parameters() override;
    controller_interface::CallbackReturn read_parameters() override;

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
  };

}  // namespace yurs_arm_controller

#endif  // YURS_ARM_CONTROLLER__YURS_ARM_CONTROLLER_HPP_