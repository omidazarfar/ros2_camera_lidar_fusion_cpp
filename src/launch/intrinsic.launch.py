from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_camera_lidar_fusion_cpp',
            executable='intrinsic_calibration_node',
            name='intrinsic_calibration_node',
            output='screen'
        )
    ])
