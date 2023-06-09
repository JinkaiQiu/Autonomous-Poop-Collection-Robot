import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    main_cont = Node(
        package='CRAP_navigation',
        executable='test_bucketAction'
    )


    grab_action_cmd = Node(
        package='serial_motor_demo',
        executable='pooping_action_server',
    )


    main_cont_delayed = TimerAction(period=5.0, actions=[main_cont])

    ld = LaunchDescription()
    ld.add_action(grab_action_cmd)
    ld.add_action(main_cont_delayed)


    return ld