#!/usr/bin/env python3
"""Trace a small XY circle with the Panda tip through MoveIt 2 (ROS 2 Humble).

Run on the sim machine next to demo.launch.py, with Isaac Playing.
Uses the current tip pose as the circle center and keeps orientation fixed.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class CartesianCurveFollower(Node):
    """Ask MoveIt to follow sampled tip poses, then execute the joint path."""

    def __init__(self):
        super().__init__("cartesian_curve_follower")
        self.declare_parameter("group_name", "panda_arm")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("ee_frame", "panda_link8")
        self.declare_parameter("radius", 0.06)
        self.declare_parameter("samples", 36)
        self.declare_parameter("max_step", 0.01)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.radius = float(self.get_parameter("radius").value)
        self.samples = int(self.get_parameter("samples").value)
        self.max_step = float(self.get_parameter("max_step").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )

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
                rclpy.spin_once(self, timeout_sec=0.1)
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
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            raise RuntimeError("GetCartesianPath returned nothing.")
        return response

    def execute_trajectory(self, trajectory):
        """Send the planned joint path to the same controllers RViz Execute uses."""
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory is missing. Are the controllers up?")

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ExecuteTrajectory goal was rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def main():
    """Look up the tip, plan a circle, execute it through MoveIt."""
    rclpy.init(args=sys.argv)
    node = CartesianCurveFollower()
    try:
        center = node.lookup_current_tip_pose()
        node.get_logger().info(
            f"Circle center ({node.base_frame}): "
            f"x={center.position.x:.3f} y={center.position.y:.3f} z={center.position.z:.3f}"
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
        result = node.execute_trajectory(cartesian.solution)
        node.get_logger().info(f"Execute done, error={result.error_code.val}")
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
