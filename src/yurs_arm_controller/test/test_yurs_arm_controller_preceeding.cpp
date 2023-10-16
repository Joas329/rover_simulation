// Copyright (c) 2023, York University Robotics Society
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "test_yurs_arm_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using yurs_arm_controller::CMD_MY_ITFS;
using yurs_arm_controller::control_mode_type;
using yurs_arm_controller::STATE_MY_ITFS;

class YursArmControllerTest : public YursArmControllerFixture<TestableYursArmController>
{
};

TEST_F(YursArmControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->params_.joints.empty());
  ASSERT_TRUE(controller_->params_.state_joints.empty());
  ASSERT_TRUE(controller_->params_.interface_name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->params_.joints, testing::ElementsAreArray(joint_names_));
  ASSERT_THAT(controller_->params_.state_joints, testing::ElementsAreArray(state_joint_names_));
  ASSERT_THAT(controller_->state_joints_, testing::ElementsAreArray(state_joint_names_));
  ASSERT_EQ(controller_->params_.interface_name, interface_name_);
}

TEST_F(YursArmControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], state_joint_names_[i] + "/" + interface_name_);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
