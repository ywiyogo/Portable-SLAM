import os
from math import pi
from launch import LaunchDescription
from launch.event_handlers import OnProcessStart

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess

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
    madgwick_config = os.path.join(package_dir, "config", "imu_filter_madgwick.yaml")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Add in generate_launch_description before creating nodes
    cleanup_dds = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /tmp/fastdds_*",
        ],
        output="screen",
        shell=True,
    )

    # Launch YDLidar node
    ydlidar_node = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        parameters=[ydlidar_params_path],
    )

    # Launch Sense Hat IMU node
    imu_sense_hat2_node = Node(
        package="portable_slam",
        executable="sense_hat_node",
        name="sense_hat_node",
        output="screen",
        parameters=[
            {
                "i2c_bus": 1,
                "use_sim_time": use_sim_time,
                "publish_rate": 10.0,  # Higher rate for better motion tracking
                "frame_id": "imu_link",
                "qos_depth": 5,  # Increased for higher frequency data
            }
        ],
    )

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        namespace="",
        parameters=[
            madgwick_config,
        ],
        output="screen",
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    launch_ekf_after_lidar_imu = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ydlidar_node, on_start=ekf_filter_node
        )
    )

    # TF2 static transforms
    # The IMU sensor of Sense HAT is rotated 180deg on Z axis
    imu_base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
    )
    # The Lidar is 4 cm higher from the base board
    lidar_base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_broadcaster",
        arguments=["0", "0", "0.4", "0", "0", "0", "base_link", "laser_frame"],
    )

    ld = LaunchDescription()
    ld.add_action(cleanup_dds)

    ld.add_action(declare_use_sim_time_argument)

    ld.add_action(imu_sense_hat2_node)
    ld.add_action(imu_base_transform)
    ld.add_action(lidar_base_transform)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(ydlidar_node)
    ld.add_action(launch_ekf_after_lidar_imu)

    return ld
