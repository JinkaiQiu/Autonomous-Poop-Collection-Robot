import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription



pkg_share = launch_ros.substitutions.FindPackageShare(package='CRAP_navigation').find('CRAP_navigation')

def generate_launch_description():

    # EKF launching
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # SLAM launching
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time': 'true','params_file': 'config/mapper_params_online_async.yaml'}.items()
    )

    return LaunchDescription([
            slam,
            # robot_localization_node

        ])