import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml


from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    # *********************** GAZEBO *********************** # 
    
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    
    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'panda'],
                        output='screen')
    
    # *************** ROBOT DESCRIPTION *************** #

    # PANDA ROBOT Description file package:
    pkg_path = os.path.join(
        get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

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

    # Hand Controller
    hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
    )

    # *********************** MoveIt!2 *********************** # 

    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_moveit_config").to_moveit_configs()

    return LaunchDescription(
        [

            # Gazebo nodes:
            gazebo, 
            spawn_entity,
            generate_static_virtual_joint_tfs_launch(moveit_config),
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
            RegisterEventHandler(
                OnProcessExit(
                    target_action = arm_group_controller,
                    on_exit = [
                        hand_controller,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = hand_controller,
                    on_exit = [

                        # MoveIt!2:
                        TimerAction(
                            period=5.0,
                            actions=[
                                generate_moveit_rviz_launch(moveit_config),
                                generate_move_group_launch(moveit_config),
                            ]
                        ),

                    ]
                )
            )
        ]
    )

