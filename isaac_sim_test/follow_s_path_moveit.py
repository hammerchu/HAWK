#!/usr/bin/env python3
"""Trace S_path with the GCR16 joint6 tip (gcr16_link6) through MoveIt 2.

Window mesh and path share window_frame.json. Run next to duco_arm_example
demo.launch.py with Isaac Playing. Ctrl+C cancels and holds the arm.
"""

from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from window_path.cad_frame import (  # noqa: E402
    apply_window_pose,
    cad_cm_to_ros_m,
    load_window_pose,
    rpy_to_quat_xyzw,
)
from window_path.dxf_path import densify_closed, load_s_rectangle_cm  # noqa: E402


def load_s_world_waypoints(pose):
    """Build world-frame S waypoints from the live DXF and the shared parent pose."""
    corners_cm = load_s_rectangle_cm()
    cad = [cad_cm_to_ros_m(*p) for p in corners_cm]
    world = apply_window_pose(
        cad,
        pose["translation_m"],
        pose["rpy_rad"],
        pose["approach_offset_m"],
    )
    spacing = float(pose.get("spacing_m", 0.03))
    return world, densify_closed(world, spacing)


class SPathFollower(Node):
    """IK the S rectangle with MoveIt, then stream it to duco_arm_controller."""

    def __init__(self, pose):
        """Create the GCR Cartesian follower using window_frame.json MoveIt names."""
        super().__init__("gcr16_s_path_follower")
        moveit = pose.get("moveit", {})
        self.declare_parameter("group_name", moveit.get("group_name", "duco_arm"))
        self.declare_parameter("base_frame", moveit.get("base_frame", "base_link"))
        self.declare_parameter("ee_frame", moveit.get("ee_frame", "gcr16_link6"))
        self.declare_parameter(
            "controller_action",
            moveit.get("controller_action", "/duco_arm_controller/follow_joint_trajectory"),
        )
        self.declare_parameter("max_step", 0.02)
        self.declare_parameter("duration_sec", 36.0)
        self.declare_parameter("avoid_collisions", False)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.controller_action = self.get_parameter("controller_action").value
        self.max_step = float(self.get_parameter("max_step").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        self.pose = pose
        self.corners_world, self.waypoints_world = load_s_world_waypoints(pose)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._static_tf = StaticTransformBroadcaster(self)
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.follow_client = ActionClient(self, FollowJointTrajectory, self.controller_action)
        self._goal_handle = None
        self._stop_requested = False
        self._last_joint_state = None
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self._exec_event_pub = self.create_publisher(String, "/trajectory_execution_event", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/hawk/s_path", 10)
        self._marker_timer = self.create_timer(0.5, self._publish_markers)
        self._publish_window_tf()
        signal.signal(signal.SIGINT, self._on_stop_signal)
        signal.signal(signal.SIGTERM, self._on_stop_signal)

    def _publish_window_tf(self):
        """Broadcast world -> window_frame so RViz and the path share one parent."""
        tx, ty, tz = self.pose["translation_m"]
        qx, qy, qz, qw = rpy_to_quat_xyzw(*self.pose["rpy_rad"])
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pose.get("parent_frame", "world")
        msg.child_frame_id = "window_frame"
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = tz
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self._static_tf.sendTransform(msg)

    def _on_joint_state(self, msg):
        """Cache the latest arm state so a stop can hold the current joints."""
        self._last_joint_state = msg

    def _on_stop_signal(self, _signum, _frame):
        """Set the stop flag only; cancel runs on the main loop (signal-safe)."""
        self._stop_requested = True

    def request_stop(self):
        """Set the stop flag; cancel runs on the main loop."""
        self._stop_requested = True

    def cancel_active_goal(self):
        """Ask duco_arm_controller to abort the current FollowJointTrajectory."""
        if self._goal_handle is None:
            return
        try:
            cancel_future = self._goal_handle.cancel_goal_async()
            self.wait_for_future(cancel_future, timeout_sec=2.0)
        except Exception as exc:
            self.get_logger().warn(f"Cancel failed: {exc}")
        self._goal_handle = None

    def publish_moveit_stop(self):
        """Tell MoveIt's execution manager to halt (same idea as the RViz Stop button)."""
        msg = String()
        msg.data = "stop"
        for _ in range(5):
            self._exec_event_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def hold_current_joints(self, timeout_sec=2.0):
        """Preempt the arm controller with a 1-point hold at the latest /joint_states."""
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while self._last_joint_state is None and rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        js = self._last_joint_state
        if js is None or not js.name:
            self.get_logger().warn("No /joint_states yet; cannot send a hold.")
            return
        if not self.follow_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(f"{self.controller_action} not up; cannot send a hold.")
            return
        traj = JointTrajectory()
        traj.joint_names = list(js.name)
        point = JointTrajectoryPoint()
        point.positions = list(js.position)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.2 * 1e9)
        traj.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        send_future = self.follow_client.send_goal_async(goal)
        self.wait_for_future(send_future, timeout_sec=3.0)

    def stop_motion_now(self):
        """Cancel any live goal, stop MoveIt execution, and hold the current pose."""
        self.publish_moveit_stop()
        self.cancel_active_goal()
        self.hold_current_joints()
        self.get_logger().info("Stop sent.")

    def wait_for_future(self, future, timeout_sec=None):
        """Spin in small slices so Ctrl+C is handled in the foreground."""
        deadline = None
        if timeout_sec is not None:
            deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and not future.done() and not self._stop_requested:
            if deadline is not None and self.get_clock().now() > deadline:
                return future
            rclpy.spin_once(self, timeout_sec=0.1)
        return future

    def lookup_current_tip_pose(self, timeout_sec=5.0):
        """Read the current joint6 tip pose in the robot base frame from TF."""
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.ee_frame, rclpy.time.Time()
                )
                pose = Pose()
                pose.position.x = tf.transform.translation.x
                pose.position.y = tf.transform.translation.y
                pose.position.z = tf.transform.translation.z
                pose.orientation = tf.transform.rotation
                return pose
            except Exception:
                if self._stop_requested:
                    raise KeyboardInterrupt
                rclpy.spin_once(self, timeout_sec=0.1)
        if self._stop_requested:
            raise KeyboardInterrupt
        raise RuntimeError(
            f"No TF from {self.base_frame} to {self.ee_frame}. "
            "Is duco_arm_example demo.launch.py up?"
        )

    def _publish_markers(self):
        """Draw the S loop and glass quad in RViz so they sit on the same pane."""
        self._publish_window_tf()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.pose.get("parent_frame", "world")
        line = Marker()
        line.header = header
        line.ns = "s_path"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.012
        line.color = ColorRGBA(r=0.15, g=0.95, b=0.35, a=1.0)
        line.pose.orientation.w = 1.0
        for x, y, z in self.waypoints_world:
            line.points.append(Point(x=x, y=y, z=z))
        glass = Marker()
        glass.header = header
        glass.ns = "s_glass"
        glass.id = 2
        glass.type = Marker.TRIANGLE_LIST
        glass.action = Marker.ADD
        glass.scale.x = 1.0
        glass.scale.y = 1.0
        glass.scale.z = 1.0
        glass.color = ColorRGBA(r=0.4, g=0.75, b=1.0, a=0.25)
        glass.pose.orientation.w = 1.0
        c0, c1, c2, c3 = self.corners_world
        for a, b, c in ((c0, c1, c2), (c0, c2, c3)):
            glass.points.append(Point(x=a[0], y=a[1], z=a[2]))
            glass.points.append(Point(x=b[0], y=b[1], z=b[2]))
            glass.points.append(Point(x=c[0], y=c[1], z=c[2]))
        self._marker_pub.publish(MarkerArray(markers=[line, glass]))

    def sample_path_waypoints(self, tip_orientation: Quaternion):
        """Stamp S world positions with a fixed joint6 orientation."""
        waypoints = []
        for x, y, z in self.waypoints_world:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation = tip_orientation
            waypoints.append(pose)
        return waypoints

    def compute_cartesian_path(self, waypoints):
        """Ask MoveIt to IK the waypoint list into a joint trajectory."""
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/compute_cartesian_path is missing. Is move_group running?")

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.ee_frame
        request.waypoints = waypoints
        request.max_step = self.max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = self.avoid_collisions

        future = self.cartesian_client.call_async(request)
        self.wait_for_future(future, timeout_sec=30.0)
        if self._stop_requested:
            raise KeyboardInterrupt
        response = future.result()
        if response is None:
            raise RuntimeError("GetCartesianPath returned nothing.")
        return response

    def stretch_joint_trajectory(self, joint_traj, duration_sec):
        """Re-time points so the whole path lasts duration_sec; drop vel/acc."""
        stretched = JointTrajectory()
        stretched.header = joint_traj.header
        stretched.joint_names = list(joint_traj.joint_names)
        count = len(joint_traj.points)
        if count == 0:
            raise RuntimeError("Cartesian path had no joint points.")
        for i, src in enumerate(joint_traj.points):
            point = src
            t = 0.0 if count == 1 else duration_sec * i / (count - 1)
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            point.velocities = []
            point.accelerations = []
            stretched.points.append(point)
        return stretched

    def execute_on_arm_controller(self, robot_trajectory):
        """Send a slow joint path to duco_arm_controller, same action as RViz Execute."""
        joint_traj = self.stretch_joint_trajectory(
            robot_trajectory.joint_trajectory, self.duration_sec
        )
        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"{self.controller_action} is missing. Is duco_arm_controller spawned?"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        self.get_logger().info(
            f"Sending {len(joint_traj.points)} pts over {self.duration_sec:.1f}s "
            f"to {self.controller_action}"
        )
        send_future = self.follow_client.send_goal_async(goal)
        self.wait_for_future(send_future, timeout_sec=10.0)
        if self._stop_requested:
            raise KeyboardInterrupt
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("FollowJointTrajectory goal was rejected.")

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self.wait_for_future(result_future, timeout_sec=self.duration_sec + 5.0)
        if self._stop_requested:
            self.stop_motion_now()
            raise KeyboardInterrupt
        self._goal_handle = None
        return result_future.result().result


def main():
    """Preview the aligned S loop, or plan it and execute on the GCR tip."""
    parser = argparse.ArgumentParser(description="GCR joint6 tip follows S_path.")
    parser.add_argument("--stop", action="store_true", help="Halt leftover motion only.")
    parser.add_argument("--preview", action="store_true", help="Publish RViz markers, no motion.")
    parser.add_argument("--pose", default="", help="Override window_frame.json")
    args, ros_args = parser.parse_known_args()

    pose = load_window_pose(args.pose or None)
    rclpy.init(args=ros_args)
    node = SPathFollower(pose)
    cmin = [min(p[i] for p in node.corners_world) for i in range(3)]
    cmax = [max(p[i] for p in node.corners_world) for i in range(3)]
    node.get_logger().info(
        f"S world x[{cmin[0]:.3f},{cmax[0]:.3f}] "
        f"y[{cmin[1]:.3f},{cmax[1]:.3f}] z[{cmin[2]:.3f},{cmax[2]:.3f]} "
        f"n={len(node.waypoints_world)} ee={node.ee_frame} group={node.group_name}"
    )
    node.get_logger().info("RViz: add MarkerArray /hawk/s_path. Green loop must sit on the pane.")

    if args.stop:
        node.get_logger().info("Stop-only mode.")
        node.stop_motion_now()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    if args.preview:
        node.get_logger().info("Preview — Ctrl+C to quit. No motion.")
        try:
            while rclpy.ok() and not node._stop_requested:
                rclpy.spin_once(node, timeout_sec=0.2)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    node.get_logger().info("Foreground — Ctrl+C stops the script and the arm.")
    try:
        tip = node.lookup_current_tip_pose()
        node.get_logger().info(
            f"Tip {node.ee_frame} in {node.base_frame}: "
            f"x={tip.position.x:.3f} y={tip.position.y:.3f} z={tip.position.z:.3f} "
            "(orientation kept for the whole S loop — jog the wrist to face the glass first)"
        )
        waypoints = node.sample_path_waypoints(tip.orientation)
        cartesian = node.compute_cartesian_path(waypoints)
        node.get_logger().info(
            f"Cartesian fraction={cartesian.fraction:.2f} error={cartesian.error_code.val}"
        )
        if cartesian.fraction < 0.99:
            node.get_logger().error(
                "MoveIt could not cover the whole S loop. Jog closer, or edit "
                "window_path/window_frame.json translation_m and re-run."
            )
            return
        result = node.execute_on_arm_controller(cartesian.solution)
        node.get_logger().info(f"FollowJointTrajectory error_code={result.error_code}")
    except KeyboardInterrupt:
        node.stop_motion_now()
        node.get_logger().info("Stopped by Ctrl+C.")
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
