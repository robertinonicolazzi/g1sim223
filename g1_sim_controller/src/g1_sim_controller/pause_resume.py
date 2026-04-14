#!/usr/bin/env python3
"""
G1 Pause/Resume Node  (ROS 2 Humble + Nav2)
============================================
Subscribes to /inorbit/custom_command (std_msgs/String).
  - "pause"  → saves current goal, publishes zero cmd_vel,
                pauses Nav2 lifecycle manager
  - "resume" → resumes Nav2 lifecycle manager, re-sends saved goal

Dependencies:
  - ROS 2 Humble + Nav2
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ---------------------------------------------------------------------------
# ManageLifecycleNodes command constants (Humble)
# ---------------------------------------------------------------------------
PAUSE  = ManageLifecycleNodes.Request.PAUSE   # = 1
RESUME = ManageLifecycleNodes.Request.RESUME  # = 2

LIFECYCLE_SERVICE  = "/lifecycle_manager_navigation/manage_nodes"
NAVIGATE_TO_POSE   = "navigate_to_pose"

RESUME_DELAY_SEC   = 1.5   # wait for Nav2 to fully activate before re-sending goal


class G1PauseResumeNode(Node):

    def __init__(self):
        super().__init__("g1_pause_resume")

        # ── State ────────────────────────────────────────────────────────────
        self._paused              = False
        self._saved_goal          = None   # PoseStamped saved before pause
        self._current_goal        = None   # PoseStamped from latest NavigateToPose feedback
        self._current_goal_handle = None   # Active Nav2 goal handle (for cancellation)

        # ── Nav2 lifecycle service client ────────────────────────────────────
        self._lc_client = self.create_client(ManageLifecycleNodes, LIFECYCLE_SERVICE)
        if not self._lc_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"Lifecycle manager '{LIFECYCLE_SERVICE}' not available yet."
            )

        # ── Nav2 NavigateToPose action client (for re-sending goal on resume)
        self._nav_client = ActionClient(self, NavigateToPose, NAVIGATE_TO_POSE)

        # ── Subscribe to NavigateToPose goal topic to track active goal ──────
        self._goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self._goal_pose_cb,
            10,
        )

        # Also track goal from NavigateToPose feedback (more reliable)
        self._feedback_timer = None

        # ── cmd_vel publisher (used to stop the robot) ───────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Command subscriber ───────────────────────────────────────────────
        self._cmd_sub = self.create_subscription(
            String,
            "/inorbit/custom_command",
            self._cmd_callback,
            10,
        )

        self.get_logger().info(
            "G1 pause/resume node ready. "
            "Listening on /inorbit/custom_command ..."
        )

    # =========================================================================
    # Goal tracking
    # =========================================================================

    def _goal_pose_cb(self, msg: PoseStamped):
        """Track the latest goal pose published to /goal_pose (from RViz or application)."""
        if not self._paused:
            self._current_goal = msg
            self.get_logger().debug(
                f"Goal updated: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
            )

    def set_current_goal(self, pose: PoseStamped):
        """Called externally (or from your mission node) to register the active goal."""
        if not self._paused:
            self._current_goal = pose

    # =========================================================================
    # Command callback
    # =========================================================================

    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()

        if cmd == "PAUSE_NAVIGATION":
            if self._paused:
                self.get_logger().info("Already paused — ignoring.")
                return
            self.get_logger().info("PAUSE_NAVIGATION command received.")
            self._do_pause()

        elif cmd == "RESUME_NAVIGATION":
            if not self._paused:
                self.get_logger().info("Not paused — ignoring.")
                return
            self.get_logger().info("RESUME_NAVIGATION command received.")
            self._do_resume()

        elif cmd == "STOP_NAVIGATION":
            self.get_logger().info("STOP_NAVIGATION command received.")
            self._do_stop()

        else:
            self.get_logger().warn(
                f"Unknown command '{msg.data}'. Valid: "
                "'PAUSE_NAVIGATION' | 'RESUME_NAVIGATION' | 'STOP_NAVIGATION'"
            )

    # =========================================================================
    # Pause
    # =========================================================================

    def _do_pause(self):
        # 1. Save goal BEFORE pausing so we can restore it
        if self._current_goal is not None:
            self._saved_goal = self._current_goal
            self.get_logger().info(
                f"Goal saved: ({self._saved_goal.pose.position.x:.2f}, "
                f"{self._saved_goal.pose.position.y:.2f})"
            )
        else:
            self.get_logger().warn(
                "No active goal tracked — resume will not re-send a goal. "
                "Make sure your mission publishes to /goal_pose."
            )

        # 2. Stop the robot immediately via Unitree SDK
        self._stop_move()

        # 3. Pause Nav2 lifecycle (deactivates controller, planner, costmaps)
        self._call_lifecycle(PAUSE, on_done=None)

        self._paused = True
        self.get_logger().info("Robot PAUSED.")

    # =========================================================================
    # Resume
    # =========================================================================

    def _do_resume(self):
        # 1. Resume Nav2 lifecycle
        self._call_lifecycle(RESUME, on_done=self._on_nav2_resumed)

    def _on_nav2_resumed(self, success: bool):
        if not success:
            self.get_logger().error("Nav2 RESUME failed — not re-sending goal.")
            return

        self._paused = False
        self.get_logger().info("Nav2 RESUMED.")

        # 2. Wait briefly for Nav2 nodes to fully activate, then re-send goal
        if self._saved_goal is not None:
            self.get_logger().info(
                f"Waiting {RESUME_DELAY_SEC}s before re-sending goal ..."
            )
            self._feedback_timer = self.create_timer(
                RESUME_DELAY_SEC, self._resend_goal_timer_cb
            )
        else:
            self.get_logger().warn("No saved goal to re-send.")

    def _resend_goal_timer_cb(self):
        # One-shot timer
        self._feedback_timer.cancel()
        self._feedback_timer = None
        self._resend_goal(self._saved_goal)

    def _resend_goal(self, pose: PoseStamped):
        """Send the saved goal to NavigateToPose action server."""
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"Re-sending goal: ({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f})"
        )

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb,
        )
        send_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal was REJECTED by Nav2.")
            return
        self._current_goal_handle = goal_handle
        self.get_logger().info("NavigateToPose goal ACCEPTED — robot is navigating.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        # Update current goal from action feedback so pause always has latest pose
        fb = feedback_msg.feedback
        if not self._paused and hasattr(fb, "current_pose"):
            # keep _current_goal as the destination, not current position
            pass

    def _nav_result_cb(self, future):
        result = future.result()
        status = result.status
        self._current_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation goal SUCCEEDED.")
            self._saved_goal    = None
            self._current_goal  = None
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation goal CANCELED.")
        else:
            self.get_logger().warn(f"Navigation goal ended with status: {status}")

    # =========================================================================
    # Stop
    # =========================================================================

    def _do_stop(self):
        # 1. Stop the robot immediately via Unitree SDK
        self._stop_move()

        # 2. Cancel the active Nav2 goal if one is running
        if self._current_goal_handle is not None:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._nav_cancel_cb)
        else:
            self.get_logger().info("No active Nav2 goal to cancel.")

        # 3. Clear all goal state — robot stays ready for new goals
        self._saved_goal          = None
        self._current_goal        = None
        self._current_goal_handle = None
        self._paused              = False
        self.get_logger().info("Navigation STOPPED.")

    def _nav_cancel_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Nav2 goal cancel response: {result}")
        except Exception as e:
            self.get_logger().warn(f"Nav2 goal cancel failed: {e}")

    # =========================================================================
    # Helpers
    # =========================================================================

    def _stop_move(self):
        """Publish zero velocity to /cmd_vel to stop the robot."""
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info("StopMove: published zero cmd_vel")

    def _call_lifecycle(self, command: int, on_done=None):
        """Async call to Nav2 lifecycle manager."""
        cmd_name = "PAUSE" if command == PAUSE else "RESUME"

        if not self._lc_client.service_is_ready():
            self.get_logger().error(
                f"Lifecycle service not ready — cannot send {cmd_name}."
            )
            if on_done:
                on_done(False)
            return

        req = ManageLifecycleNodes.Request()
        req.command = command
        future = self._lc_client.call_async(req)
        future.add_done_callback(
            lambda f: self._lifecycle_response_cb(f, cmd_name, on_done)
        )

    def _lifecycle_response_cb(self, future, cmd_name: str, on_done=None):
        try:
            resp = future.result()
            ok = resp.success
            if ok:
                self.get_logger().info(f"Lifecycle {cmd_name}: success")
            else:
                self.get_logger().warn(f"Lifecycle {cmd_name}: returned failure")
        except Exception as e:
            self.get_logger().error(f"Lifecycle {cmd_name} call failed: {e}")
            ok = False

        if on_done:
            on_done(ok)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = G1PauseResumeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()