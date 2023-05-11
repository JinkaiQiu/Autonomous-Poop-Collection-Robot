import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import time
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='CRAP_sim_def').find('CRAP_sim_def')
    default_model_path = os.path.join(pkg_share, 'urdf/CRAP.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')
    world_file_path = 'worlds/TestRoom.world'
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

    diff_drive_spawner = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["diff_cont"],
    )
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(pkg_share,'config','controller_actual.yaml')

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    joint_broad_spawner = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_broad"],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    #use_sim_time_sub = LaunchConfiguration('use_sim_time')

    rviz_node_delayed = TimerAction(period=3.0, actions=[rviz_node])
    controller_manager_delayed = TimerAction(period=3.0, actions=[controller_manager])
    diff_drive_spawner_delayed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )
    diff_drive_spawner_delayed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner_delayed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        controller_manager_delayed,
        diff_drive_spawner_delayed,
        joint_broad_spawner_delayed,
        
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                    description='Absolute path to rviz config file'),
        rviz_node_delayed,
    ])