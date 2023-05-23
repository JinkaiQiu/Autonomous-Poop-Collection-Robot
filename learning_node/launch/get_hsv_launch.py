from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument that can be passed to the script
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),

        # Include kinect2_bridge kinect2_bridge.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/kinect2_ros2/kinect2_bridge/launch/kinect2_bridge.launch.py']),
            launch_arguments={
                'sensor': '008190334247',
                'use_sim_time': use_sim_time
            }.items(),
        ),

        # Node for learning_node node_object_1pp
        Node(
            package='learning_node',
            executable='get_hsv',
            name='get_hsv',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
