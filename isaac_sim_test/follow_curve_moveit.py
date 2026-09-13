#!/usr/bin/env python3
"""Trace a small, slow XY circle with the Panda tip (ROS 2 Humble).

Plans with MoveIt Cartesian IK, then sends the path to panda_arm_controller
(FollowJointTrajectory) — the same pipe RViz Execute uses for Isaac.
"""

import math
import signal
import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory


class CartesianCurveFollower(Node):
    """Sample a tip curve, IK it, then stream a slow trajectory to the arm controller."""

    def __init__(self):
        super().__init__("cartesian_curve_follower")
        self.declare_parameter("group_name", "panda_arm")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("ee_frame", "panda_link8")
        self.declare_parameter("controller_action", "/panda_arm_controller/follow_joint_trajectory")
        self.declare_parameter("radius", 0.04)
        self.declare_parameter("samples", 48)
        self.declare_parameter("max_step", 0.01)
        self.declare_parameter("duration_sec", 8.0)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.controller_action = self.get_parameter("controller_action").value
        self.radius = float(self.get_parameter("radius").value)
        self.samples = int(self.get_parameter("samples").value)
        self.max_step = float(self.get_parameter("max_step").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.follow_client = ActionClient(
            self, FollowJointTrajectory, self.controller_action
        )
        self._goal_handle = None
        self._stop_requested = False
        signal.signal(signal.SIGINT, self._on_sigint)

    def _on_sigint(self, _signum, _frame):
        """Set the stop flag only; cancel runs on the main loop (signal-safe)."""
        self._stop_requested = True

    def cancel_active_goal(self):
        """Ask panda_arm_controller to abort the current FollowJointTrajectory."""
        if self._goal_handle is None:
            return
        try:
            cancel_future = self._goal_handle.cancel_goal_async()
            self.wait_for_future(cancel_future, timeout_sec=2.0)
        except Exception as exc:
            self.get_logger().warn(f"Cancel failed: {exc}")
        self._goal_handle = None

    def wait_for_future(self, future, timeout_sec=None):
        """Spin in small slices so Ctrl+C is handled in the foreground (not a daemon)."""
        deadline = None
        if timeout_sec is not None:
            deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and not future.done() and not self._stop_requested:
            if deadline is not None and self.get_clock().now() > deadline:
                return future
            rclpy.spin_once(self, timeout_sec=0.1)
        return future

    def lookup_current_tip_pose(self, timeout_sec=5.0):
        """Read the current tip pose in the robot base frame from TF."""
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
            "Is demo.launch.py up, and are the frame names right?"
        )

    def sample_circle_waypoints(self, center):
        """Build a closed XY circle around center; Z and orientation stay fixed."""
        waypoints = []
        for i in range(self.samples + 1):
            angle = 2.0 * math.pi * i / self.samples
            pose = Pose()
            pose.position.x = center.position.x + self.radius * math.cos(angle)
            pose.position.y = center.position.y + self.radius * math.sin(angle)
            pose.position.z = center.position.z
            pose.orientation = center.orientation
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
        request.avoid_collisions = True

        future = self.cartesian_client.call_async(request)
        self.wait_for_future(future, timeout_sec=15.0)
        if self._stop_requested:
            raise KeyboardInterrupt
        response = future.result()
        if response is None:
            raise RuntimeError("GetCartesianPath returned nothing.")
        return response

    def stretch_joint_trajectory(self, joint_traj, duration_sec):
        """Re-time points so the whole path lasts duration_sec; drop vel/acc for smooth interp."""
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
        """Send a slow joint path to panda_arm_controller, same action RViz Execute uses."""
        joint_traj = self.stretch_joint_trajectory(
            robot_trajectory.joint_trajectory, self.duration_sec
        )
        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"{self.controller_action} is missing. "
                "Is panda_arm_controller spawned?"
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
            self.cancel_active_goal()
            raise KeyboardInterrupt
        self._goal_handle = None
        return result_future.result().result


def main():
    """Look up the tip, plan a small circle, execute slowly on the arm controller."""
    rclpy.init(args=sys.argv)
    node = CartesianCurveFollower()
    node.get_logger().info("Foreground process — Ctrl+C stops the script and cancels the arm.")
    try:
        center = node.lookup_current_tip_pose()
        node.get_logger().info(
            f"Circle center ({node.base_frame}): "
            f"x={center.position.x:.3f} y={center.position.y:.3f} z={center.position.z:.3f} "
            f"radius={node.radius:.3f}m"
        )
        waypoints = node.sample_circle_waypoints(center)
        cartesian = node.compute_cartesian_path(waypoints)
        node.get_logger().info(
            f"Cartesian fraction={cartesian.fraction:.2f} "
            f"error={cartesian.error_code.val}"
        )
        if cartesian.fraction < 0.99:
            node.get_logger().error(
                "MoveIt could not cover the whole curve. "
                "Shrink radius, or start from a more reachable pose."
            )
            return
        result = node.execute_on_arm_controller(cartesian.solution)
        node.get_logger().info(f"FollowJointTrajectory error_code={result.error_code}")
    except KeyboardInterrupt:
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
