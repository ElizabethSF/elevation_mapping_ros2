from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    demo_pkg = get_package_share_directory('elevation_mapping_demos')

    fused_config = os.path.join(demo_pkg, 'config', 'visualization', 'fused.yaml')
    raw_config = os.path.join(demo_pkg, 'config', 'visualization', 'raw.yaml')

    return LaunchDescription([
        # elevation_map fused visualization
        Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
            name='elevation_map_fused_visualization',
            output='screen',
            parameters=[fused_config, {'grid_map_topic': '/elevation_mapping/elevation_map'}],
        ),

        # elevation_map raw visualization
        Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
            name='elevation_map_raw_visualization',
            output='screen',
            parameters=[raw_config, {'grid_map_topic': '/elevation_mapping/elevation_map_raw'}],
        ),
    ])
