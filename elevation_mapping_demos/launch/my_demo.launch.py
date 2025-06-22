from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    demo_pkg = get_package_share_directory('elevation_mapping_demos')

    robot_config = os.path.join(demo_pkg, 'config', 'robots', 'my_demo_robot.yaml')
    map_config = os.path.join(demo_pkg, 'config', 'elevation_maps', 'my_demo_map.yaml')
    postprocess_config = os.path.join(demo_pkg, 'config', 'postprocessing', 'postprocessor_pipeline.yaml')
    rviz_config = os.path.join(demo_pkg, 'rviz', 'elevation_map_visualization.rviz')
    visualization_launch = os.path.join(demo_pkg, 'launch', 'visualization.launch.py')  # 你也得把 visualization.launch 改成 .py

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Elevation Mapping Node
        Node(
            package='elevation_mapping',
            executable='elevation_mapping',
            name='elevation_mapping',
            output='screen',
            # parameters=[robot_config, map_config, postprocess_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            parameters=[robot_config, map_config, postprocess_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Include Visualization Launch File (must be .launch.py too)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch)
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/snoopy/Autowheelloader/ws_elevation_map_ros2/src/elevation_mapping_ros2/elevation_mapping_demos/rviz/my_demo.rviz'],
            # output='screen'
        )
    ])
