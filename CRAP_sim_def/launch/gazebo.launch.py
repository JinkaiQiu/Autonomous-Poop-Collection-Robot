import os
import launch
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

from launch_ros.actions import Node


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='CRAP_sim_def' #<--- CHANGE ME
    world_file_path = 'worlds/TestRoom.world'
    default_rviz_config_path = 'rviz/urdf.rviz'
    config_file_path = 'config/ekf.yaml'
    model = 'urdf/CRAP.xacro'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)  
    config_path = os.path.join(pkg_path, config_file_path)  
    default_model_path = os.path.join(pkg_path, model)  

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    
    crap = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','CRAP.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'world':world_path}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'crap',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')
    
    
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
 

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Launch them all!
    return LaunchDescription([
        crap,
        # rviz_arg,
        gazebo,
        spawn_entity,
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        #robot_localization_node,
        # rviz_node       
    ])