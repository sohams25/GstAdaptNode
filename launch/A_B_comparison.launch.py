"""
A/B Stress Test: Two separate containers for independent CPU measurement.
Each has its own MediaStreamerNode (10 fps cap to prevent PC overload).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    video_path_arg = DeclareLaunchArgument(
        'video_path', default_value='/tmp/test_video.mp4')

    video_path = LaunchConfiguration('video_path')

    legacy_container = ComposableNodeContainer(
        name='legacy_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gst_adapt_node',
                plugin='gst_adapt_node::MediaStreamerNode',
                name='legacy_source',
                parameters=[{
                    'video_path': video_path,
                    'image_topic': '/legacy/image_raw',
                    'info_topic': '/legacy/camera_info',
                    'max_fps': 10.0,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::ResizeNode',
                name='legacy_resize',
                remappings=[
                    ('image/image_raw', '/legacy/image_raw'),
                    ('image/camera_info', '/legacy/camera_info'),
                    ('resize/image_raw', '/legacy/image_processed'),
                    ('resize/camera_info', '/legacy/resize_camera_info'),
                ],
                parameters=[{
                    'use_scale': False,
                    'height': 480,
                    'width': 640,
                    'interpolation': 1,
                }],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    accel_container = ComposableNodeContainer(
        name='accel_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gst_adapt_node',
                plugin='gst_adapt_node::MediaStreamerNode',
                name='accel_source',
                parameters=[{
                    'video_path': video_path,
                    'image_topic': '/accelerated/image_raw',
                    'info_topic': '/accelerated/camera_info',
                    'max_fps': 10.0,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='gst_adapt_node',
                plugin='gst_adapt_node::ResizeNode',
                name='accel_resize',
                parameters=[{
                    'input_topic': '/accelerated/image_raw',
                    'output_topic': '/accelerated/image_processed',
                    'action': 'resize',
                    'source_width': 3840,
                    'source_height': 2160,
                    'use_scale': False,
                    'height': 480,
                    'width': 640,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    latency = Node(
        package='gst_adapt_node',
        executable='latency_tracker.py',
        name='latency_tracker',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        video_path_arg,
        legacy_container,
        accel_container,
        latency,
    ])
