import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



pkg_share = launch_ros.substitutions.FindPackageShare(package='CRAP_navigation').find('CRAP_navigation')

def generate_launch_description():
   use_sim_time = LaunchConfiguration('use_sim_time')

   teleop_node = Node(
      package='teleop_twist_keyboard',
      executable='teleop_twist_keyboard',
      name='teleop_node',
      parameters=[{'use_sim_time': use_sim_time}],
      remappings=[('/cmd_vel','/cmd_vel_key')]
    )
   
   twist_mux_node = Node(
      package='twist_mux',
      executable='twist_mux',
      parameters= [os.path.join(pkg_share, 'config/twist_mux.yaml')],
      remappings=[('/cmd_vel_out','diff_cont/cmd_vel_unstamped')]
   )

   return LaunchDescription([
      DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
      twist_mux_node,
      # teleop_node     
   ])