from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_camera_lidar_fusion_cpp',
            executable='sensor_data_saver_node',
            name='sensor_data_saver_node',
            parameters=[{"save_dir": "data"}],
            output='screen'
        )
    ])
