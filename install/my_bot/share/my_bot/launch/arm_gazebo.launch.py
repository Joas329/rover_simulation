import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    
    #Gazebo Launch File
    gazebo_params_file = os.path.join(get_package_share_directory("my_bot"),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    #Robot Description & Robot State Publisher

    xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(),'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    #Spawn Robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    #return 
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])