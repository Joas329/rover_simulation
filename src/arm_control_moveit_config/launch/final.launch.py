import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():


    # *********************** GAZEBO *********************** # 
    
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    # *************** ROBOT DESCRIPTION *************** #

    # PANDA ROBOT Description file package:
    pkg_path = os.path.join(
        get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','arm_gazebo.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}


    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'arm_bot'],
                        output='screen')
    
    # *************** STATIC TRANSFORM *************** #

    # NODE -> Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0,0","0.0", "0.0", "0.0", "world", "base_link"],
    )
    # Publish TF:
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )

    # *************** ROS2_CONTROLLERS *************** #

     # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm Group Controller
    arm_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "-c", "/controller_manager"],
    )

    # # Hand Controller
    # hand_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["hand_controller", "-c", "/controller_manager"],
    # )

    # *********************** MoveIt!2 *********************** # 

    # *** PLANNING CONTEXT *** #

    # Robot description, SRDF:
    robot_description_semantic_config = load_file("arm_control_moveit_config", "config/robot_arm.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config }
    
    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("arm_control_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # ompl_planning_yaml = load_yaml("arm_control_moveit_config", "config/ompl_planning.yaml")
    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "monitor_dynamics": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
    }

    moveit_config = (
        MoveItConfigsBuilder("arm_control")
        .robot_description(
            file_path="config/robot_arm.urdf.xacro",
            # mappings={
            #     "ros2_control_hardware_type": LaunchConfiguration(
            #         "ros2_control_hardware_type"
            #     )
            # },
        )
        .robot_description_semantic(file_path="config/robot_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters= [moveit_config.to_dict(),move_group_configuration,],
        arguments=["--ros-args", "--log-level", "info"],
    )



    # ******************* RVIZ2 *******************#
    
    rviz_base = os.path.join(get_package_share_directory("arm_control_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )


    # *******************JOY**********************#
    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            # ComposableNode(
            #     package="tf2_ros",
            #     plugin="tf2_ros::StaticTransformBroadcasterNode",
            #     name="static_tf2_broadcaster",
            #     parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
            # ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )



    return LaunchDescription(
        [

            # Gazebo nodes:
            gazebo, 
            spawn_entity,
            # ROS2_CONTROL:
            static_tf,
            robot_state_publisher,
            
            # ROS2 Controllers:
            RegisterEventHandler(
                OnProcessExit(
                    target_action = spawn_entity,
                    on_exit = [
                        joint_state_broadcaster,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_state_broadcaster,
                    on_exit = [
                        arm_group_controller,
                    ]
                )
            ),
            # RegisterEventHandler(
            #     OnProcessExit(
            #         target_action = arm_group_controller,
            #         on_exit = [
            #             hand_controller,
            #         ]
            #     )
            # ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = arm_group_controller,
                    on_exit = [

                        # MoveIt!2:
                        TimerAction(
                            period=5.0,
                            actions=[
                                rviz_node_full,
                                move_group_node,
                                container
                            ]
                        ),

                    ]
                )
            )
        ]
    )