"""
A/B Stress Test: Legacy CPU resize vs. Hardware-Accelerated GStreamer resize.
Both pipelines use zero-copy intra-process media sources.

ROS Graph:
  legacy_container (intra-process)         accel_container (intra-process)
  +-------------------------------+        +-------------------------------+
  | MediaStreamerNode              |        | MediaStreamerNode              |
  |   /legacy/image_raw      ----+--+      |   /accelerated/image_raw ----+--+
  |   /legacy/camera_info         |  |     |   /accelerated/camera_info   |  |
  +-------------------------------+  |     +-------------------------------+  |
  | image_proc::ResizeNode        |  |     | gst_adapt_node::GstAdaptNode |  |
  |   <- /legacy/image_raw   <---+  |     |   <- /accelerated/image_raw<-+  |
  |   -> /legacy/image_processed -+--+-->  |   -> /accel/image_processed -+--+-->
  +-------------------------------+  |     +-------------------------------+  |
                                     v                                        v
                   +------------------------------------------+
                   |         latency_tracker (Python)         |
                   |   /legacy/image_processed                |
                   |   /accelerated/image_processed           |
                   +------------------------------------------+
                   |         cpu_monitor (Python)              |
                   |   Tracks CPU% per container process       |
                   +------------------------------------------+
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='/tmp/test_video.mp4',
        description='Path to input video file (set empty to use synthetic 4K)')

    video_path = LaunchConfiguration('video_path')

    # -----------------------------------------------------------------------
    # PIPELINE A — Legacy CPU resize (image_proc::ResizeNode)
    #   Requires: sudo apt install ros-humble-image-proc
    # -----------------------------------------------------------------------
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
                    'interpolation': 1,  # cv::INTER_LINEAR
                }],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    # -----------------------------------------------------------------------
    # PIPELINE B — Hardware-accelerated resize (gst_adapt_node)
    # -----------------------------------------------------------------------
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
                    'format': 'yuv420p',
                    'image_topic': '/accelerated/image_raw',
                    'info_topic': '/accelerated/camera_info',
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='gst_adapt_node',
                plugin='gst_adapt_node::GstAdaptNode',
                name='accel_resize',
                parameters=[{
                    'input_topic': '/accelerated/image_raw',
                    'output_topic': '/accelerated/image_processed',
                    'action': 'resize',
                    'source_width': 3840,
                    'source_height': 2160,
                    'target_width': 640,
                    'target_height': 480,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    # -----------------------------------------------------------------------
    # TELEMETRY
    # -----------------------------------------------------------------------
    latency = Node(
        package='gst_adapt_node',
        executable='latency_tracker.py',
        name='latency_tracker',
        output='screen',
        emulate_tty=True,
    )

    cpu = Node(
        package='gst_adapt_node',
        executable='cpu_monitor.py',
        name='cpu_monitor',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        video_path_arg,
        legacy_container,
        accel_container,
        latency,
        cpu,
    ])
