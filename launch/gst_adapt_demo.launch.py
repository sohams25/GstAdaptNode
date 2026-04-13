import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('gst_adapt_node')
    params_file = os.path.join(pkg_dir, 'config', 'demo_params.yaml')

    container = ComposableNodeContainer(
        name='gst_adapt_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gst_adapt_node',
                plugin='gst_adapt_node::ResizeNode',
                name='resize_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
