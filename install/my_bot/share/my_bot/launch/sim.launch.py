import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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
    gazebo_params_file = os.path.join(get_package_share_directory("my_bot"),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file '+ gazebo_params_file}.items()
             )
    
   
    xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(),'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controllers"]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control','load_controller','--set-state', 'active','joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trojectory_controller=ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controllers'],
        output="screen"    
    )
    
    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_arm_controller],
        #     )
        # ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    
    ])

