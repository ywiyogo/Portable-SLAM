import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Get YDLidar params file path
    ydlidar_params_path = PathJoinSubstitution(
        [FindPackageShare("ydlidar_ros2_driver"), "params", "TminiPro.yaml"]
    )

    package_dir = get_package_share_directory("portable_slam")
    ekf_config = os.path.join(package_dir, "config", "imu_odom_config.yaml")

    slam_toolbox_params_config = os.path.join(
        package_dir, "config", "mapper_params_online_async.yaml"
    )

    return LaunchDescription(
        [
            # Launch argument declarations
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            # Launch YDLidar node
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node",
                output="screen",
                parameters=[ydlidar_params_path],
            ),
            # Launch Sense Hat IMU node
            Node(
                package="portable_slam",
                executable="sense_hat_node",
                name="sense_hat_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "publish_rate": 100.0,
                        "frame_id": "imu_link",
                        "i2c_bus": 5,
                    }
                ],
            ),
            # robot_localization uses the IMU data to estimate the robot's pose
            TimerAction(
                period=1.0,  # Delay for 5 seconds (adjust as needed)
                actions=[
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="ekf_filter_node",
                        output="screen",
                        parameters=[ekf_config],
                    ),
                ],
            ),
            # Launch SLAM Toolbox
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[
                            slam_toolbox_params_config,
                        ],
                        arguments=["--ros-args", "--log-level", "debug"]
                    ),
                ],
            ),
            # TF2 static transforms
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_broadcaster",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="laser_broadcaster",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser_frame"],
            ),
        ]
    )
