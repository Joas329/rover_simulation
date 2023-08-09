#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>


// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();


class PoseSub : public rclcpp::Node
{
public:
    PoseSub()
        : Node("posesub")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pose", 10, std::bind(&PoseSub::poseCallback, this, std::placeholders::_1));

        // Create the MoveIt MoveGroup Interface
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");
    }

private:
    void poseCallback()
    {
        RCLCPP_INFO(get_logger(), "I heard: %.2f, %.2f, %.2f", inmsg->orientation.x, inmsg->orientation.y, inmsg->orientation.z);
        // Set a target Pose
        move_group_interface_->setPoseTarget(target_pose);

        // Create a plan to that target pose
        auto const [success, plan] = move_group_interface_->plan();

        // Execute the plan
        if (success)
        {
            move_group_interface_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Planning failed!");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSub>();

    poseCallback();

    // Shutdown ROS
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
