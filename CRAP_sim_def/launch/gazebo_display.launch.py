import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import time
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='CRAP_sim_def').find('CRAP_sim_def')
    default_model_path = os.path.join(pkg_share, 'urdf/CRAP.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')
    world_file_path = 'worlds/largerRoom.world'
    # world_file_path = 'worlds/TestRoom.world'
    world_path=os.path.join(pkg_share, world_file_path),

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'crap', '-topic', 'robot_description'],
        output='screen',
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    diff_drive_spawner = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["diff_cont"],
    )

    joint_broad_spawner = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_broad"],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    #use_sim_time_sub = LaunchConfiguration('use_sim_time')

    rviz_node_delayed = TimerAction(period=2.0, actions=[rviz_node])
    robot_state_publisher_node_delayed = TimerAction(period=2.0, actions=[robot_state_publisher_node])
    joint_broad_spawner_delayed = TimerAction(period=5.0, actions=[joint_broad_spawner])
    spawn_entity_delayed = TimerAction(period=3.0, actions=[spawn_entity])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        robot_state_publisher_node,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        #robot_localization_node,

        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                    description='Absolute path to rviz config file'),
        rviz_node_delayed,
    ])