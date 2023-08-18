import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions
import launch
from launch.actions import RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml
import xacro
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

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


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='arm_control_moveit_config'

    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rsp.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'false'}.items()
    # )
    # pkg_path = os.path.join(
    #     get_package_share_directory('my_bot'))
    # xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    # robot_description = {'robot_description': robot_description_config.toxml()}

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='both',
    #     parameters=[
    #         robot_description,
    #         {"use_sim_time": True}
    #     ]
    # )



    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    # controller_params_file = os.path.join(get_package_share_directory(package_name),'config','arm_controllers.yaml')

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{'robot_description': '/robot_state_publisher'},
    #                 controller_params_file]
    # )

    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # # Arm Group Controller
    # arm_group_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["arm_group_controller", "-c", "/controller_manager"],
    # )


    # delayed_arm_group_controler = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[arm_group_controller, joint_state_broadcaster ],
    #     )
    # )


    pkg_path = os.path.join(
    get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','arm_hardware.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # pkg_path = os.path.join(
    #     get_package_share_directory('my_bot'))
    # xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    # robo

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_bot"),
            "config",
            "arm_controllers.yaml",
        ]
    )
    controller_params_file = os.path.join(get_package_share_directory("my_bot"),'config','arm_controllers.yaml')
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )


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
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
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



    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,

        RegisterEventHandler(
                OnProcessExit(
                    target_action = arm_controller_spawner,
                    on_exit = [

                        # MoveIt!2:
                        TimerAction(
                            period=5.0,
                            actions=[
                                rviz_node_full,
                                move_group_node
                            ]
                        ),

                    ]
                )
            )
        
    ]

    return LaunchDescription(nodes)