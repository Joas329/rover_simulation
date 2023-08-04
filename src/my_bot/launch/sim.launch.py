import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():


    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    gazebo_params_file = os.path.join(get_package_share_directory("my_bot"),'config','/gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file '+ gazebo_params_file}.items()
             )
    
   
    xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params,{"use_sim_tine": True}]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    # **** Load Controllers**** #

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

    
    return LaunchDescription([

        gazebo,
        node_robot_state_publisher,
        TimerAction(
            period=2.0,
            actions=[
                spawn_entity,
            ]
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[arm_group_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[hand_controller],
            )
        ),
        
    ])

