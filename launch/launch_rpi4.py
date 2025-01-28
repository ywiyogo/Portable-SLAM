import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
                "publish_rate": 20.0,  # Reduced to match EKF rate
                "frame_id": "imu_link",
                "qos_depth": 1  # Small queue size for real-time data
            }
        ],
    )

    # slam_toolbox Lifescycle Node declarations and its event triggers
    autostart = LaunchConfiguration("autostart")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
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

    localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )
    # TF2 static transforms
    imu_base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
    )
    lidar_base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser_frame"],
    )

    # robot_localization uses the IMU data to estimate the robot's pose
    start_localization_1s_delay_node = TimerAction(
        period=1.0,  # Delay for 1 second
        actions=[localization_node],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)

    ld.add_action(ydlidar_node)
    ld.add_action(imu_sense_hat2_node)
    ld.add_action(start_localization_1s_delay_node)
    ld.add_action(imu_base_transform)
    ld.add_action(lidar_base_transform)
    # slam_toolbox
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(slam_toolbox_configure_event)
    ld.add_action(slam_toolbox_activate_event)

    return ld