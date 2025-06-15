from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_camera_lidar_fusion_cpp',
            executable='projection_node',
            name='projection_node',
            parameters=[
                {"intrinsic_yaml": "config/camera_intrinsics.yaml"},
                {"extrinsic_yaml": "config/extrinsic_calib.yaml"}
            ],
            output='screen'
        )
    ])
