import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
import xacro
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot'

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


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    # return LaunchDescription([
    #     robot_state_publisher,
    #     delayed_controller_manager,
    #     delayed_arm_group_controler,
    # ])



    pkg_path = os.path.join(
    get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','arm.urdf.xacro')
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
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','arm_controllers.yaml')
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

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]

    return LaunchDescription(nodes)