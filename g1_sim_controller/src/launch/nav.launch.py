"""
nav.launch.py
=============
Launches Nav2 for the Unitree G1 humanoid robot.

Requires g1_slam to be running first:
  ros2 launch g1_slam slam.launch.py

Nav2 stack started:
  - controller_server    (DWB local planner -> /cmd_vel)
  - planner_server       (NavFn / Dijkstra global planner)
  - behavior_server      (spin, backup, wait)
  - bt_navigator         (behavior tree navigation)
  - waypoint_follower    (follows a list of waypoints)
  - velocity_smoother    (smooths cmd_vel output)
  - lifecycle_manager    (manages all Nav2 nodes)

Send a single goal from terminal:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, \
      pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}"

Arguments:
  use_sim_time   [true|false]  default: false
  params_file    path to nav2_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os


PKG = "g1_sim_controller"


def _pkg_path(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


def generate_launch_description():

    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    arg_params = DeclareLaunchArgument(
        "params_file",
        default_value=_pkg_path("config", "nav2_params.yaml"),
        description="Path to Nav2 parameters YAML file.",
    )
    arg_network_iface = DeclareLaunchArgument(
        "network_iface",
        default_value="eth0",
        description="Network interface for Unitree SDK (e.g. eth0, enp3s0).",
    )
    use_sim_time   = LaunchConfiguration("use_sim_time")
    params_file    = LaunchConfiguration("params_file")
    network_iface  = LaunchConfiguration("network_iface")

    nav2_params = [
        params_file,
        {"use_sim_time": use_sim_time},
    ]

    bt_xml = _pkg_path("config", "navigate_to_pose.xml")
    bt_navigator_params = nav2_params + [
        {
            "default_nav_to_pose_bt_xml": bt_xml,
            "default_nav_through_poses_bt_xml": bt_xml,
        }
    ]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=nav2_params,
        # remappings=[("cmd_vel", "/cmd_vel_nav")], # Publish directly to /cmd_vel
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=nav2_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        output="screen",
        parameters=nav2_params,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=bt_navigator_params
    )

    # waypoint_follower removed for lighter setup
    # smoother_server removed for lighter setup
    # velocity_smoother removed for lighter setup
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart":    True},
            {"node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ]},
        ],
    )

    pause_resume_node = Node(
        package="g1_sim_controller",
        executable="pause_resume",
        output="screen",
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_params,
        arg_network_iface,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        pause_resume_node,
    ])