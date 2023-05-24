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
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    python_commander_dir = get_package_share_directory('nav2_simple_commander')
    pkgShare_dir = launch_ros.substitutions.FindPackageShare(package='CRAP_navigation').find('CRAP_navigation')

    # map_yaml_file = os.path.join(pkgShare_dir, 'maps', 'my_map_save.yaml')
    map_yaml_file = os.path.join(pkgShare_dir, 'maps', 'biggerMAP.yaml')
    nav_yaml_file = os.path.join(pkgShare_dir, 'config', 'nav2_params.yaml')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'slam': 'False','use_sim_time': 'false', 'params_file': nav_yaml_file, 'map': map_yaml_file}.items())
    
    
    main_cont = Node(
        package='CRAP_navigation',
        executable='main_controller'
    )


    grab_action_cmd = Node(
        package='serial_motor_demo',
        executable='pooping_action_server',
    )


    main_cont_delayed = TimerAction(period=5.0, actions=[main_cont])

    ld = LaunchDescription()
    ld.add_action(bringup_cmd)
    ld.add_action(grab_action_cmd)
    ld.add_action(main_cont_delayed)


    return ld