import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare("portable_slam")
    config_file = PathJoinSubstitution(
        [pkg_share, "config", "imu_filter_madgwick.yaml"]
    )

    return LaunchDescription(
        [
            # Launch the IMU data publisher
            Node(
                package="portable_slam",
                executable="sense_hat_node",
                name="sense_hat_node",
                output="screen",
                parameters=[
                    {
                        "publish_rate": 50.0,  # Match the IMU filter rate (1/constant_dt)
                        "frame_id": "imu_link",
                        "i2c_bus": 1,
                        "latency_threshold": 0.1,
                    }
                ],
            ),
            # Launch the IMU filter with debug configuration
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_broadcaster",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
            ),
        ]
    )
