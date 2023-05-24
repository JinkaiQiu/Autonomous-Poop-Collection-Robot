import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import time
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # declare variables
    pkg_share = launch_ros.substitutions.FindPackageShare(package='CRAP_sim_def').find('CRAP_sim_def')
   
    # Rviz Node
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Lidar Node
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    lidar_parameter_file = LaunchConfiguration('lidar_params_file')

    lidar_driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                            executable='ydlidar_ros2_driver_node',
                            name='ydlidar_ros2_driver_node',
                            output='screen',
                            emulate_tty=True,
                            parameters=[lidar_parameter_file],
                            namespace='/',
                            )

    # ROS2 control nodes
    controller_params_file = os.path.join(pkg_share,'config','real_controllers.yaml')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("CRAP_sim_def"), "urdf", "CRAP_real.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_manager = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="both",
        parameters=[robot_description], 
    )

    joint_broad_spawner = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        # arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["diff_cont", "--controller-manager", "/controller_manager"],
    )


    # Kinect Node
    kinect = launch_ros.actions.Node(
        package='learning_node',
        executable='node_object_1pp',
        name='node_object_1pp',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'),


    # Twist mux node
    pkg_share_nav = launch_ros.substitutions.FindPackageShare(package='CRAP_navigation').find('CRAP_navigation')
    twist_mux_node = launch_ros.actions.Node(
      package='twist_mux',
      executable='twist_mux',
      parameters= [os.path.join(pkg_share_nav, 'config/twist_mux.yaml')],
      remappings=[('/cmd_vel_out','diff_cont/cmd_vel_unstamped')]
   )

    # Delayed actions
    rviz_node_delayed = TimerAction(period=3.0, actions=[rviz_node])

    diff_drive_spawner_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    kinect_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[kinect],
        )
    )

    
    return launch.LaunchDescription([   
        # launch lidar node
        
        launch.actions.DeclareLaunchArgument('lidar_params_file',
                                           default_value=os.path.join(
                                            share_dir, 'params', 'X3.yaml'),
                                           description='FPath to the ROS2 parameters file to use.'),
        lidar_driver_node,
       
        # launch ros2 control nodes
        controller_manager,
        robot_state_publisher_node,
        joint_broad_spawner,
        diff_drive_spawner_delayed,

        # launch twist mux
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        twist_mux_node,

        # launch rviz
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                    description='Absolute path to rviz config file'),
        rviz_node_delayed,

        #launch kinect
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/kinect2_ros2/kinect2_bridge/launch/kinect2_bridge.launch.py']),
            launch_arguments={
                'sensor': '008190334247',
                'use_sim_time': use_sim_time
            }.items(),
        ),
        kinect_delayed,

    ])