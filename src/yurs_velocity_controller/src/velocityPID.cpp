#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "std_msgs/msg/header.hpp"

class VelPID : public rclcpp::Node
{
    public:
    VelPID(): Node("VelPID")
    {
        // *****Declare Node Variables Here******* 
        i = 0;
        reading_time = 0.0;
        trajectory_flag = false;
        flag = false;
        stack = std::vector<moveit_msgs::msg::RobotTrajectory>();

        // ********Timers********
        velocity_PID_timer = create_wall_timer(std::chrono::milliseconds(20), std::bind(&VelPID::velocity_PID, this))
        trajectory_indexing_timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&VelPID::trajectory_indexing, this));
        clip_publisher = create_wall_timer(std::chrono::seconds(3), std::bind(&VelPID::clip_publisher_callback, this));

        // ********Subscribers********
        joint_trajectory_subscriber = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "display_planned_path",
            10,
            std::bind(&joint_trajectory_callback, this, std::placeholders::_1));

        joint_state_subscriber = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&position_feedback_callback, this, std::placeholders::_1));

        // ***********Publishers***************
        effort_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            "arm_group_controller/commands",
            10);

        // ********** Tunning Publishers ***********
        debug_desired_pose = create_publisher<std_msgs::msg::Float64MultiArray>(
            "veloctiy_PID/desired_pose",
            10);

        debug_current_pose = create_publisher<std_msgs::msg::Float64MultiArray>(
            "velocity_PID/current_pose",
            10);

        debug_desired_velocity = create_publisher<std_msgs::msg::Float64MultiArray>(
            "effort_PID/desired_velocity",
            10);

        debug_current_velocity = create_publisher<std_msgs::msg::Float64MultiArray>(
            "effort_PID/current_velocity",
            10);
        
    }

    private:
    int i;
    bool trajectory_flag;
    bool flag;
    double reading_time;
    std::vector<moveit_msgs::msg::DisplayTrajectory> stack;
    std::vector<double> ordered_current_pose, 
                        current_position, 
                        desired_joint_positions, 
                        command, 
                        velocity, 
                        curr_vel, 
                        prev_curr_vel, 
                        joint_desired_velocity,
                        effort;
    std::vector<std::string> joint_names;
    std::vector<int> joint_order;

    // Timers
    rclcpp::TimerBase::SharedPtr velocity_PID_timer;
    rclcpp::TimerBase::SharedPtr trajectory_indexing_timer;
    rclcpp::TimerBase::SharedPtr clip_publisher;

    // Subscribers
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr joint_trajectory_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_desired_pose;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_current_pose;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_desired_velocity;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_current_velocity;


    // **********Subcriber Callbacks***********
    void joint_trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
    {
        stack = std::vector(std::begin(msg->trajectory), std::end(msg->trajectory));
        trajectory_flag = false;
        i = 0;
    }

    void position_feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!flag)
        {
            // Runs once at the beginning of the code as the joint states are published immediately.
            joints_number = msg->name.size(); //checks for number of detected joints
            joint_names = msg->name;
            joint_order = std::vector(joints_number, 0);

            // Velocity
            ordered_current_pose = std::vector(joints_number, 0.0);
            current_position = std::vector(joints_number, 0.0);
            desired_joint_positions = std::vector(joints_number, 0.0);
            command = std::vector(joints_number, 0.0);
            velocity = std::vector(joints_number, 0.0);

            // Effort
            ordered_current_pose = std::vector(msg->name.size(), 0.0);
            curr_vel = std::vector(msg->name.size(), 0.0);
            prev_curr_vel = std::vector(msg->name.size(), 0.0);
            joint_desired_velocity = std::vector(msg->name.size(), 0.0);
            effort = std::vector(msg->name.size(), 0.0);

            // Parameter Definition
            declare_parameter("k_pose_p", std::vector(joints_number, 0.0));
            declare_parameter("k_pose_i", std::vector(joints_number, 0.0));
            declare_parameter("k_pose_d", std::vector(joints_number, 0.0));
            declare_parameter("k_vel_p", std::vector(joints_number, 0.0));
            declare_parameter("k_vel_i", std::vector(joints_number, 0.0));
            declare_parameter("k_vel_d", std::vector(joints_number, 0.0));

            flag = true;

            dynamic_joint_check(msg->name);
        }

        reading_time = rclcpp::Clock().now().seconds();
        pose_to_vel();
        current_position = msg->position;
        for (int i = 0; i < joints_number - 1; ++i)
        {
            ordered_current_pose[i] = current_position[joint_order[i]];
        }
    }

    void velocity_PID()
    {
        if (!stack.empty())
        {
            auto velocities = pid_velocity_calc(desired_joint_positions, joints_number);
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


    void trajectory_indexing()
    {
        // This indexes through the trajectory array
        if (!stack.empty())
        {
            if (!trajectory_flag)
            {
                printf("Received Trajectory\n");
                trajectory_flag = true;
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

    void effort_callback()
    {
        // This performs pid
        if (!stack.empty())
        {
            auto velocities = pid_effort_calc(stack);

            prev_curr_vel = curr_vel;

            effort_publisher->publish(velocities);
        }
    }

    void clip_publisher_callback()
    {
        if (flag)
        {
            auto k_pose_p = get_parameter("k_pose_p").get_value<std::vector<double>>();
            auto k_pose_i = get_parameter("k_pose_i").get_value<std::vector<double>>();
            auto k_pose_d = get_parameter("k_pose_d").get_value<std::vector<double>>();
            auto k_vel_p = get_parameter("k_vel_p").get_value<std::vector<double>>();
            auto k_vel_i = get_parameter("k_vel_i").get_value<std::vector<double>>();
            auto k_vel_d = get_parameter("k_vel_d").get_value<std::vector<double>>();

            RCLCPP_INFO(get_logger(), "number of detected joints: %d", joints_number);

            // Assuming joint_names is a std::vector<std::string>
            std::string detected_joints;
            for (const auto &joint_name : joint_names)
            {
                detected_joints += joint_name + " ";
            }
            RCLCPP_INFO(get_logger(), "detected joints: %s", detected_joints.c_str());

            RCLCPP_INFO(get_logger(), "***********Velocity**********");
            RCLCPP_INFO(get_logger(), "P values: %s", vector_to_string(k_pose_p).c_str());
            RCLCPP_INFO(get_logger(), "I values: %s", vector_to_string(k_pose_i).c_str());
            RCLCPP_INFO(get_logger(), "D values: %s", vector_to_string(k_pose_d).c_str());
            RCLCPP_INFO(get_logger(), "");

            RCLCPP_INFO(get_logger(), "***********Effort**********");
            RCLCPP_INFO(get_logger(), "P values: %s", vector_to_string(k_vel_p).c_str());
            RCLCPP_INFO(get_logger(), "I values: %s", vector_to_string(k_vel_i).c_str());
            RCLCPP_INFO(get_logger(), "D values: %s", vector_to_string(k_vel_d).c_str());
            RCLCPP_INFO(get_logger(), "***********Commands**********");
            RCLCPP_INFO(get_logger(), "commands: %s", vector_to_string(command).c_str());
            RCLCPP_INFO(get_logger(), "**********Current Position*************");
            RCLCPP_INFO(get_logger(), "%s", vector_to_string(current_position).c_str());
            RCLCPP_INFO(get_logger(), "**********Desired Position*************");
            RCLCPP_INFO(get_logger(), "%s", vector_to_string(desired_joint_positions).c_str());
        }
    }

    std::string vector_to_string(const std::vector<double> &vec)
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


    std_msgs::msg::Float64MultiArray pid_velocity_calc(
        const std::vector<double> &joint_desired_positions,
        int joints_number)
    {
        std_msgs::msg::Float64MultiArray velocity_command;

        auto k_pose_p = get_parameter("k_pose_p").get_value<std::vector<double>>();
        auto k_pose_i = get_parameter("k_pose_i").get_value<std::vector<double>>();
        auto k_pose_d = get_parameter("k_pose_d").get_value<std::vector<double>>();

        int joint = 0;

        std::vector current_pose_error(joints_number, 0.1);
        std::vector velocity_sum(joints_number, 0.1);
        std::vector previous_pose_error(joints_number, 0.1);
        std::vector p_pose_factor(joints_number, 0.1);
        std::vector i_pose_factor(joints_number, 0.1);
        std::vector d_pose_factor(joints_number, 0.1);

        for (const auto &desired_pose : joint_desired_positions)
        {
            current_pose_error[joint] = desired_pose - ordered_current_pose[joint];
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
            velocity_command.data.push_back(velocity[joint]);
            ++joint;
        }

        return velocity_command;
    }

    std_msgs::msg::Float64MultiArray pid_effort_calc(
        const std::vector<double> &joint_desired_velocity,
        const int& joints_number
        )
    {
        std_msgs::msg::Float64MultiArray effort_command;

        auto k_vel_p = get_parameter("k_vel_p").get_value<std::vector<double>>();
        auto k_vel_i = get_parameter("k_vel_i").get_value<std::vector<double>>();
        auto k_vel_d = get_parameter("k_vel_d").get_value<std::vector<double>>();

        int joint = 0;

        std::vector current_vel_error(joints_number, 0.1);
        std::vector effort_sum(joints_number, 0.1);
        std::vector previous_vel_error(joints_number, 0.1);
        std::vector p_vel_factor(joints_number, 0.1);
        std::vector i_vel_factor(joints_number, 0.1);
        std::vector d_vel_factor(joints_number, 0.1);

        for (const auto &desired_velocity : joint_desired_velocity)
        {
            current_vel_error[joint] = desired_velocity - curr_vel[joint];

            p_vel_factor[joint] = k_vel_p[joint] * current_vel_error[joint];

            i_vel_factor[joint] = k_vel_i[joint] * (effort_sum[joint] + current_vel_error[joint]);
            effort_sum[joint] += current_vel_error[joint];

            d_vel_factor[joint] = k_vel_d[joint] * (current_vel_error[joint] - previous_vel_error[joint]);
            previous_vel_error[joint] = current_vel_error[joint];

            effort[joint] = p_vel_factor[joint] + i_vel_factor[joint] + d_vel_factor[joint];
            effort_command.data.push_back(effort[joint]);

            ++joint;
        }

        return effort_command;
    }


    void pose_to_vel()
    {
        double t = std::chrono::duration<double>(std::chrono::system_clock::now() - reading_time).count();

        for (size_t i = 0; i < 5; ++i)
        {
            curr_vel[i] = (curr_vel[i] - prev_curr_vel[i]) / t;
        }
    }

    void dynamic_joint_check(const std::vector<std::string> &joint_states)
    {
        int counter = 0;
        for (const auto &joint : joint_states)
        {
            auto val = std::stoi(joint.substr(joint.size() - 1)) - 1;
            joint_order[val] = counter;
            ++counter;
        }
    }
}