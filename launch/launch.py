from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Get YDLidar params file path
    ydlidar_params_path = PathJoinSubstitution(
        [FindPackageShare("ydlidar_ros2_driver"), "params", "TminiPro.yaml"]
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
            # Launch SLAM Toolbox
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "base_frame": "base_link",
                        "odom_frame": "odom",
                        "map_frame": "map",
                        "published_frame": "odom",
                        "use_pose_extrapolator": True,
                        "use_nav_sat": False,
                        "max_laser_range": 20.0,
                        "map_update_interval": 5.0,
                        "resolution": 0.05,
                        "transform_timeout": 0.2,
                        "scan_topic": "scan",  # Your LiDAR topic
                        "imu_topic": "imu/data_raw",  # Your IMU topic
                        "use_imu": True,  # Enable IMU in SLAM Toolbox
                        "imu_angle_drift": 0.001,
                        "mode": "mapping",
                    }
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
