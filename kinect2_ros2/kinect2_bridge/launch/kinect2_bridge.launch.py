import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


parameters=[    {'base_name': 'kinect2',
                'sensor': '',
                'fps_limit': -1.0,
                'use_png': False,
                'jpeg_quality': 90,
                'png_level': 1,
                'depth_method': 'default',
                'depth_device': -1,
                'reg_method': 'default',
                'reg_device': -1,
                'max_depth': 12.0,
                'min_depth': 0.1,
                'queue_size': 5,
                'bilateral_filter': True,
                'edge_aware_filter': True,
                'worker_threads': 4,
                'publish_tf': True}]

point_cloud_res='hd' # ['hd','qhd','sd']

def generate_launch_description():
    kinect2 = Node(
            package='kinect2_bridge',
            executable='kinect2_bridge',
            emulate_tty=True,
            name='kinect2_bridge',
            parameters=parameters,
            output='screen')
    

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='pointcloud',
                remappings=[
                    ("rgb/camera_info", f"/kinect2/{point_cloud_res}/camera_info"),
                    ("rgb/image_rect_color", f"/kinect2/{point_cloud_res}/image_color_rect"),
                    ("depth_registered/image_rect",f"/kinect2/{point_cloud_res}/image_depth_rect"),
                    ("points",f"/kinect2/{point_cloud_res}/points")]
            ),
        ],
    )


    return LaunchDescription([
        kinect2, container
    ])


