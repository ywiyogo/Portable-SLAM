import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    ydlidar_params_path = PathJoinSubstitution(
        [FindPackageShare("ydlidar_ros2_driver"), "params", "TminiPro.yaml"]
    )

    package_dir = get_package_share_directory("portable_slam")
    gtsam_config = os.path.join(package_dir, "config", "gtsam_slam_config.yaml")
    lps22hb_config = os.path.join(package_dir, "config", "lps22hb_config.yaml")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    ydlidar_node = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        parameters=[ydlidar_params_path],
    )

    imu_sense_hat_node = Node(
        package="portable_slam",
        executable="sense_hat_node",
        name="sense_hat_node",
        output="screen",
        parameters=[
            {
                "i2c_bus": 1,
                "use_sim_time": use_sim_time,
                "publish_rate": 20.0,
                "frame_id": "imu_link",
            }
        ],
    )

    lps22hb_node = Node(
        package="portable_slam",
        executable="lps22hb_node",
        name="lps22hb_node",
        output="screen",
        parameters=[
            lps22hb_config,
            {
                "i2c_bus": 1,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 10.0,
            }
        ],
    )

    gtsam_slam_node = Node(
        package="portable_slam",
        executable="gtsam_slam_node",
        name="gtsam_slam_node",
        output="screen",
        parameters=[
            gtsam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    imu_base_transform = Node(
        package="portable_slam",
        executable="static_transform_node",
        name="static_transform_node_imu",
        parameters=[{"parent_frame": "base_link", "child_frame": "imu_link"}],
    )

    lidar_base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_broadcaster",
        arguments=[
            "0",
            "0",
            "0.4",
            "-1.5708",
            "0",
            "0",
            "base_link",
            "laser_frame",
        ],
    )

    calibrate_imu = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=imu_sense_hat_node,
            on_start=[
                LogInfo(msg="[Calibration] Waiting 2 seconds for IMU to initialize..."),
                ExecuteProcess(cmd=["sleep", "2.0"], output="screen"),
                LogInfo(msg="[Calibration] Starting IMU calibration..."),
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/calibrate_imu",
                        "std_srvs/srv/Trigger",
                        "{}",
                    ],
                    output="screen",
                ),
                LogInfo(msg="[Calibration] IMU calibration completed"),
            ],
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(ydlidar_node)
    ld.add_action(imu_sense_hat_node)
    ld.add_action(calibrate_imu)
    ld.add_action(lps22hb_node)
    ld.add_action(rf2o_node)
    ld.add_action(gtsam_slam_node)
    ld.add_action(imu_base_transform)
    ld.add_action(lidar_base_transform)

    return ld
