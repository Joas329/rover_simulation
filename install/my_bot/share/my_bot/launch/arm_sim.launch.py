import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    # robot model to option m1013 or a0912

    robot_model = 'arm'
    # robot_model = 'm1013'

    gazebo_params_file = os.path.join(get_package_share_directory('my_bot'),'config','gazebo_params.yaml')

    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Robot State Publisher
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn the robot in Gazebo
    spawn_entity_robot = Node(package='gazebo_ros',
                              executable='spawn_entity.py',
                              arguments=['-entity', 'my_bot', '-topic', 'robot_description'],
                              output='screen')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file '+ gazebo_params_file}.items()
             )
   
   # Start Gazebo with my empty world
    world_file_name = 'my_empty_world.world'
    world = os.path.join(get_package_share_directory('my_bot'))
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        robot_state_publisher,
        spawn_entity_robot,
        gazebo])
