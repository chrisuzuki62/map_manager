from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('map_manager'),
        'cfg',
        'occupancy_map_param.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'occupancy_map_param_file',
            default_value=param_file_path,
            description='Path to the occupancy map parameter file'
        ),

        Node(
            package='map_manager',
            executable='occupancy_map_node',
            name='occupancy_map_node',
            output='screen',
            parameters=[LaunchConfiguration('occupancy_map_param_file')]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"]
        )
    ])

