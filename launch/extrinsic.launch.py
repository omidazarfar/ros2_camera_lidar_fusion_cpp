from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_camera_lidar_fusion_cpp',
            executable='extrinsic_calibration_node',
            name='extrinsic_calibration_node',
            parameters=[
                {"input_yaml": "config/point_pairs.yaml"},
                {"intrinsic_yaml": "config/camera_intrinsics.yaml"},
                {"output_yaml": "config/extrinsic_calib.yaml"}
            ],
            output='screen'
        )
    ])
