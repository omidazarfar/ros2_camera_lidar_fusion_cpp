from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_camera_lidar_fusion_cpp',
            executable='manual_point_selector_node',
            name='manual_point_selector_node',
            parameters=[
                {"image_path": "data/frame_0.png"},
                {"output_yaml": "config/point_pairs.yaml"}
            ],
            output='screen'
        )
    ])
