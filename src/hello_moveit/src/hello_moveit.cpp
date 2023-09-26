#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "manipulator");

// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}




class MultipleSubscribersNode : public rclcpp::Node {
public:
    MultipleSubscribersNode() : Node("multiple_subscribers_node") {
        // Create multiple subscribers
        subscriber1_ = this->create_subscription<std_msgs::msg::String>(
            "newPose", 10, [this](const geometry_msgs::msg::Pose msg) {
                this->handleSubscriber1(msg);
            });
    }

private:
    void handleSubscriber1(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received from subscriber 1: %s", msg->data.c_str());
    }

    void handleSubscriber2(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received from subscriber 2: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber2_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultipleSubscribersNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}