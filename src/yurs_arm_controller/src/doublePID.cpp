#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "std_msgs/msg/header.hpp"
#include <vector>

class DoublePID : public rclcpp::Node
{
public:
  DoublePID() : Node("DoublePID"), i(0), trajectoryFlag(false), readingTime(0.0), flag(false)
  {
    // Declare Node Variables Here
    stack = std::vector<moveit_msgs::msg::RobotTrajectory>();

    // Timers
    velocityPIDTimer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&DoublePID::velocityPID, this));
    trajectoryIndexingTimer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DoublePID::trajectoryIndexing, this));
    clipPublisher = this->create_wall_timer(std::chrono::seconds(3), std::bind(&DoublePID::cli_publisher_callback, this));

    // Subscribers
    jointTrajectorySubscriber = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
        "display_planned_path",
        10,
        std::bind(&DoublePID::joint_trajectory_callback, this, std::placeholders::_1));

    jointStateSubscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(&DoublePID::position_feedback_callback, this, std::placeholders::_1));

    // Publishers
    effort_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "arm_group_controller/commands",
        10);

    debug_desired_pose = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "veloctiyPID/desired_pose",
        10);

    debug_current_pose = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "velocityPID/current_pose",
        10);

    debug_desired_velocity = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "effortPID/desired_velocity",
        10);

    debug_current_velocity = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "effortPID/current_velocity",
        10);
  }

private:
  // Declare class variables here
  int i;
  std::vector<moveit_msgs::msg::RobotTrajectory> stack;
  std::vector<double> ordered_current_pose,
                      currentPosition, 
                      desired_joint_positions, 
                      command, 
                      velocity, 
                      curr_vel, 
                      prev_curr_vel, 
                      jointDesiredVelocity;
  std::vector<double> effort;
  bool trajectoryFlag;
  double readingTime;
  bool flag;
  int joints_number;
  std::vector<std::string> joint_names;
  std::vector<int> joint_order;

  // Timers
  rclcpp::TimerBase::SharedPtr velocityPIDTimer;
  rclcpp::TimerBase::SharedPtr trajectoryIndexingTimer;
  rclcpp::TimerBase::SharedPtr clipPublisher;

  // Subscribers
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr jointTrajectorySubscriber;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_desired_pose;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_current_pose;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_desired_velocity;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_current_velocity;

  // Callbacks
    void joint_trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
    {
        stack = std::vector(std::begin(msg->trajectory), std::end(msg->trajectory));
        trajectoryFlag = false;
        i = 0;
    }

    void position_feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!flag)
        {
            // Runs once at the beginning of the code as the joint states are published immediately.
            joints_number = msg->name.size(); //checks for number of detected joints
            joint_names = msg->name;
            joint_order = std::vector<int>(joints_number, 0);

            // Velocity
            ordered_current_pose = currentPosition = desired_joint_positions = std::vector<double>(joints_number, 0.0);
            command = velocity = std::vector<double>(joints_number, 0.0);

            // Effort
            ordered_current_pose = curr_vel = prev_curr_vel = jointDesiredVelocity = std::vector<double>(msg->name.size(), 0.0);
            effort = std::vector<double>(msg->name.size(), 0.0);

            // Parameter Definition
            this->declare_parameter("k_pose_p", std::vector<double>(joints_number, 0.0));
            this->declare_parameter("k_pose_i", std::vector<double>(joints_number, 0.0));
            this->declare_parameter("k_pose_d", std::vector<double>(joints_number, 0.0));
            this->declare_parameter("k_vel_p", std::vector<double>(joints_number, 0.0));
            this->declare_parameter("k_vel_i", std::vector<double>(joints_number, 0.0));
            this->declare_parameter("k_vel_d", std::vector<double>(joints_number, 0.0));

            flag = true;

            dynamicJointCheck(msg->name);
        }

        readingTime = rclcpp::Clock().now().seconds();
        poseToVel();
        currentPosition = msg->position;
        for (int i = 0; i < joints_number - 1; ++i)
        {
            ordered_current_pose[i] = currentPosition[joint_order[i]];
        }
    }


    void velocityPID()
    {
        if (!stack.empty())
        {
            auto velocities = pidVelocityCalc(desired_joint_positions, joints_number);
            effort_publisher->publish(velocities);
            command = velocities;

            // Publish Tuning Values

            // velocity_publisher->publish(velocities);

            std_msgs::msg::Float64MultiArray desired;
            desired.data = desired_joint_positions;
            debug_desired_pose->publish(desired);

            std_msgs::msg::Float64MultiArray current;
            current.data = ordered_current_pose;
            debug_current_pose->publish(current);
        }
    }


    void trajectoryIndexing()
    {
        // This indexes through the trajectory array
        if (!stack.empty())
        {
            if (!trajectoryFlag)
            {
                printf("Received Trajectory\n");
                trajectoryFlag = true;
            }

            if (stack[0]->joint_trajectory.points.size() - 1 > i)
            {
                auto trajectories = stack[0];
                auto holder = trajectories->joint_trajectory;
                auto holder2 = holder.points;

                desired_joint_positions = holder2[i].positions;
                ++i;
            }
        }
    }

    void effortCallback()
    {
        // This performs pid
        if (!stack.empty())
        {
            auto velocities = pidEffortCalc(stack);

            prev_curr_vel = curr_vel;

            effort_publisher->publish(velocities);
        }
    }



    void cli_publisher_callback()
    {
        if (flag)
        {
            auto k_pose_p = this->get_parameter("k_pose_p").get_value<std::vector<double>>();
            auto k_pose_i = this->get_parameter("k_pose_i").get_value<std::vector<double>>();
            auto k_pose_d = this->get_parameter("k_pose_d").get_value<std::vector<double>>();
            auto k_vel_p = this->get_parameter("k_vel_p").get_value<std::vector<double>>();
            auto k_vel_i = this->get_parameter("k_vel_i").get_value<std::vector<double>>();
            auto k_vel_d = this->get_parameter("k_vel_d").get_value<std::vector<double>>();

            RCLCPP_INFO(get_logger(), "number of detected joints: %d", joints_number);

            // Assuming joint_names is a std::vector<std::string>
            std::string detected_joints;
            for (const auto &joint_name : joint_names)
            {
                detected_joints += joint_name + " ";
            }
            RCLCPP_INFO(get_logger(), "detected joints: %s", detected_joints.c_str());

            RCLCPP_INFO(get_logger(), "***********Velocity**********");
            RCLCPP_INFO(get_logger(), "P values: %s", vectorToString(k_pose_p).c_str());
            RCLCPP_INFO(get_logger(), "I values: %s", vectorToString(k_pose_i).c_str());
            RCLCPP_INFO(get_logger(), "D values: %s", vectorToString(k_pose_d).c_str());
            RCLCPP_INFO(get_logger(), "");

            RCLCPP_INFO(get_logger(), "***********Effort**********");
            RCLCPP_INFO(get_logger(), "P values: %s", vectorToString(k_vel_p).c_str());
            RCLCPP_INFO(get_logger(), "I values: %s", vectorToString(k_vel_i).c_str());
            RCLCPP_INFO(get_logger(), "D values: %s", vectorToString(k_vel_d).c_str());
            RCLCPP_INFO(get_logger(), "***********Commands**********");
            RCLCPP_INFO(get_logger(), "commands: %s", vectorToString(command).c_str());
            RCLCPP_INFO(get_logger(), "**********Current Position*************");
            RCLCPP_INFO(get_logger(), "%s", vectorToString(currentPosition).c_str());
            RCLCPP_INFO(get_logger(), "**********Desired Position*************");
            RCLCPP_INFO(get_logger(), "%s", vectorToString(desired_joint_positions).c_str());
        }
    }

    std::string vectorToString(const std::vector<double> &vec)
    {
        std::ostringstream oss;
        oss << "[ ";
        for (const auto &value : vec)
        {
            oss << value << " ";
        }
        oss << "]";
        return oss.str();
    }


    std_msgs::msg::Float64MultiArray pidVelocityCalc(
        const std::vector<double> &jointDesiredPositions,
        int joints_number)
    {
        std_msgs::msg::Float64MultiArray velocityCommand;

        auto k_pose_p = this->get_parameter("k_pose_p").get_value<std::vector<double>>();
        auto k_pose_i = this->get_parameter("k_pose_i").get_value<std::vector<double>>();
        auto k_pose_d = this->get_parameter("k_pose_d").get_value<std::vector<double>>();

        int joint = 0;

        std::vector<double> current_pose_error(joints_number, 0.1);
        std::vector<double> velocity_sum(joints_number, 0.1);
        std::vector<double> previous_pose_error(joints_number, 0.1);
        std::vector<double> p_pose_factor(joints_number, 0.1);
        std::vector<double> i_pose_factor(joints_number, 0.1);
        std::vector<double> d_pose_factor(joints_number, 0.1);

        for (const auto &desiredPose : jointDesiredPositions)
        {
            current_pose_error[joint] = desiredPose - ordered_current_pose[joint];
            // P function
            p_pose_factor[joint] = k_pose_p[joint] * current_pose_error[joint];
            // I function
            i_pose_factor[joint] = k_pose_i[joint] * (velocity_sum[joint] + current_pose_error[joint]);
            velocity_sum[joint] += current_pose_error[joint];
            // D function
            d_pose_factor[joint] = k_pose_d[joint] * (current_pose_error[joint] - previous_pose_error[joint]);
            previous_pose_error[joint] = current_pose_error[joint];
            // publish this value to the respective joint
            velocity[joint] = p_pose_factor[joint] + i_pose_factor[joint] + d_pose_factor[joint];
            velocityCommand.data.push_back(velocity[joint]);
            ++joint;
        }

        return velocityCommand;
    }

    std_msgs::msg::Float64MultiArray pidEffortCalc(
        const std::vector<double> &jointDesiredVelocity,
        int joints_number
        )
    {
        std_msgs::msg::Float64MultiArray effortCommand;

        auto k_vel_p = this->get_parameter("k_vel_p").get_value<std::vector<double>>();
        auto k_vel_i = this->get_parameter("k_vel_i").get_value<std::vector<double>>();
        auto k_vel_d = this->get_parameter("k_vel_d").get_value<std::vector<double>>();

        int joint = 0;

        std::vector<double> current_vel_error(joints_number, 0.1);
        std::vector<double> effort_sum(joints_number, 0.1);
        std::vector<double> previous_vel_error(joints_number, 0.1);
        std::vector<double> p_vel_factor(joints_number, 0.1);
        std::vector<double> i_vel_factor(joints_number, 0.1);
        std::vector<double> d_vel_factor(joints_number, 0.1);

        for (const auto &desiredVelocity : jointDesiredVelocity)
        {
            current_vel_error[joint] = desiredVelocity - curr_vel[joint];

            p_vel_factor[joint] = k_vel_p[joint] * current_vel_error[joint];

            i_vel_factor[joint] = k_vel_i[joint] * (effort_sum[joint] + current_vel_error[joint]);
            effort_sum[joint] += current_vel_error[joint];

            d_vel_factor[joint] = k_vel_d[joint] * (current_vel_error[joint] - previous_vel_error[joint]);
            previous_vel_error[joint] = current_vel_error[joint];

            effort[joint] = p_vel_factor[joint] + i_vel_factor[joint] + d_vel_factor[joint];
            effortCommand.data.push_back(effort[joint]);

            ++joint;
        }

        return effortCommand;
    }


    void poseToVel()
    {
        auto current_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        double t = current_time - readingTime;

        for (size_t i = 0; i < 5; ++i)
        {
            curr_vel[i] = (curr_vel[i] - prev_curr_vel[i]) / t;
        }
    }

    void dynamicJointCheck(const std::vector<std::string> &jointStates)
    {
        int counter = 0;
        for (const auto &joint : jointStates)
        {
            joint_order[std::stoi(joint.substr(joint.size() - 1)) - 1] = counter;
            ++counter;
        }
    }



};

int main(int argc, char **argv)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing PID");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoublePID>());
  rclcpp::shutdown();

  return 0;
}
