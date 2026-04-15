from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_g1_sim_controller = FindPackageShare('g1_sim_controller')

    # Wall-clock time everywhere — the simulator runs near real-time and
    # all DDS timestamps (lidar, IMU) already use time.time().
    use_sim_time = False

    pc_config = PathJoinSubstitution([
        pkg_g1_sim_controller,
        "config",
        "pc_to_laserscan.yaml",
    ])

    slam_config = PathJoinSubstitution([
        pkg_g1_sim_controller,
        "config",
        "slam_toolbox.yaml",
    ])

    slam_localize_config = PathJoinSubstitution([
        pkg_g1_sim_controller,
        "config",
        "slam_toolbox_localize.yaml",
    ])

    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='lo',
        description='Network interface to use for Unitree SDK'
    )

    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='true',
        description='Enable SLAM mapping mode'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Enable SLAM localization mode'
    )

    mapping = LaunchConfiguration('mapping')
    localization = LaunchConfiguration('localization')

    basic_telemetry_node = Node(
        package='g1_sim_controller',
        executable='basic_telemetry',
        name='basic_telemetry',
        output='screen',
        parameters=[
            {'network_interface': LaunchConfiguration('network_interface')},
            {'use_sim_time': use_sim_time},
        ]
    )

    dds_ros2_bridge_node = Node(
        package='g1_sim_controller',
        executable='dds_ros2_bridge',
        name='dds_ros2_bridge',
        output='screen',
        parameters=[
            {'network_interface': LaunchConfiguration('network_interface')},
            {'use_sim_time': use_sim_time},
        ]
    )

    camera_bridge_node = Node(
        package='g1_sim_controller',
        executable='camera_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pose_publisher_node = Node(
        package='g1_sim_controller',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pc_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pc_to_ls",
        output="screen",
        parameters=[
            pc_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", "/utlidar/cloud_livox_mid360"),
            ("scan",     "/scan"),
        ],
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(mapping),
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    slam_node_loc = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(localization),
        parameters=[
            slam_localize_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Wait for /scan to be published before launching Nav2
    nav_launch_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'nav.launch.py',
    )
    wait_for_scan = ExecuteProcess(
        cmd=["bash", "-c", "until ros2 topic echo --once /scan > /dev/null 2>&1; do sleep 1; done"],
        output="screen",
        name="wait_for_scan",
    )
    nav2_after_scan = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch_dir),
                    launch_arguments={'use_sim_time': str(use_sim_time)}.items(),
                ),
            ],
        )
    )

    return LaunchDescription([
        network_interface_arg,
        mapping_arg,
        localization_arg,
        basic_telemetry_node,
        dds_ros2_bridge_node,
        camera_bridge_node,
        pose_publisher_node,
        pc_to_laserscan_node,
        slam_node,
        slam_node_loc,
        wait_for_scan,
        nav2_after_scan,
    ])
