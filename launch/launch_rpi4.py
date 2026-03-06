import os
from math import pi
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.actions import Node, LifecycleNode
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Define a generate_launch_description function that returns the LaunchDescription"""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Get YDLidar params file path
    ydlidar_params_path = PathJoinSubstitution(
        [FindPackageShare("ydlidar_ros2_driver"), "params", "TminiPro.yaml"]
    )

    package_dir = get_package_share_directory("portable_slam")
    ekf_config = os.path.join(package_dir, "config", "imu_odom_config.yaml")
    madgwick_config = os.path.join(package_dir, "config", "imu_filter_madgwick.yaml")

    # Launch argument declarations
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
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
                "publish_rate": 20.0,  # Higher rate for better motion tracking
                "frame_id": "imu_link",
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

    # slam_toolbox Lifescycle Node declarations and its event triggers
    autostart = LaunchConfiguration("autostart")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the slamtoolbox. "
        "Ignored when use_lifecycle_manager is true.",
    )
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        description="Enable bond connection during node activation",
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            get_package_share_directory("portable_slam"),
            "config",
            "mapper_params_online_async.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )

    start_async_slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            slam_params_file,
            {
                "use_lifecycle_manager": use_lifecycle_manager,
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
        namespace="",
    )

    slam_toolbox_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    slam_toolbox_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            start_async_slam_toolbox_node
                        ),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        emulate_tty=True,
        parameters=[ekf_config],
        # for odom input of slam_toolbox.
        # alternative would be assigning odom_topic: "/odometry/filtered" in the config file
        remappings=[("/odometry/filtered", "/odom")],
    )

    # TF2 static transforms
    imu_base_transform = Node(
        package="portable_slam",
        executable="static_transform_node",
        name="static_transform_node_imu",
        parameters=[{'parent_frame': 'base_link', 'child_frame': 'imu_link'}]
    )
    # The Lidar is 4 cm higher from the base board
    lidar_base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_broadcaster",
        arguments=["0", "0", "0.4", "-1.5708", "0", "0", "base_link", "laser_frame"],
        # -1.5708 radians = -90 degrees rotation around X axis (clockwise)
    )
    # Alternative method to estimate odometry from lidar scan.
    # However, rf2o doesn'T support IMU integration
    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": False,  # Disable TF publishing to avoid conflicts
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 10.0,
            }
        ],
    )

    launch_slam_after_ekf = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ydlidar_node,
            on_start=[
                start_async_slam_toolbox_node,
                slam_toolbox_configure_event,
                slam_toolbox_activate_event,
            ],
        )
    )

    # robot_localization uses the IMU data to estimate the robot's pose
    launch_rf2o_after_lidar_imu = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=ydlidar_node, on_start=rf2o_node)
    )

    # IMU calibration sequence - run calibration after IMU node starts
    calibrate_imu = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=imu_sense_hat2_node,
            on_start=[
                LogInfo(msg="[Calibration] Waiting 2 seconds for IMU to initialize..."),
                ExecuteProcess(
                    cmd=['sleep', '2.0'],
                    output='screen'
                ),
                LogInfo(msg="[Calibration] Starting IMU calibration..."),
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/calibrate_imu', 'std_srvs/srv/Trigger', '{}'],
                    output='screen'
                ),
                LogInfo(msg="[Calibration] IMU calibration completed, starting filter...")
            ]
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(ekf_filter_node)
    ld.add_action(imu_sense_hat2_node)
    ld.add_action(calibrate_imu)  # Add calibration sequence
    ld.add_action(imu_base_transform)
    ld.add_action(lidar_base_transform)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(ydlidar_node)
    ld.add_action(launch_rf2o_after_lidar_imu)

    # slam_toolbox
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(launch_slam_after_ekf)

    # Shutdown event handlers
    def on_shutdown(event, context):
        return [
            LogInfo(msg="[Shutdown] Initiating graceful shutdown sequence..."),
            # First stop regular nodes and services
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/ydlidar_ros2_driver_node/stop_scan', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            ),
            # Small delay to ensure lidar stops properly
            ExecuteProcess(
                cmd=['sleep', '0.5'],
                output='screen'
            ),
            # Reset lidar more thoroughly
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/ydlidar_ros2_driver_node/reset', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/ekf_filter_node/reset', 'std_srvs/srv/Empty', '{}'],
                output='screen'
            ),
            # Then handle lifecycle node shutdown
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN,
                ),
                condition=IfCondition(
                    LaunchConfiguration('use_lifecycle_manager', default='false')
                )
            ),
            LogInfo(msg="[Shutdown] Shutdown sequence complete")
        ]

    # Enhanced shutdown handler with map persistence
    def enhanced_on_shutdown(event, context):
        map_save_actions = [
            LogInfo(msg="[Map Persistence] Saving current map..."),
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/slam_toolbox/save_map', 'slam_toolbox/srv/SaveMap',
                     '{name: {data: \"/home/ubuntu/portable_slam_map\"}}'],
                output='screen'
            ),
            LogInfo(msg="[Map Persistence] Map saved successfully")
        ]
        # Get original shutdown actions and prepend map saving
        original_actions = on_shutdown(event, context)
        return map_save_actions + original_actions

    shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=enhanced_on_shutdown)
    )
    ld.add_action(shutdown_handler)

    return ld
