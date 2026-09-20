#!/usr/bin/env python3
"""Trace S_path with the GCR16 joint6 tip (gcr16_link6) through MoveIt 2.

Window mesh and path share window_frame.json. Run next to duco_arm_example
demo.launch.py with Isaac Playing. Default: joint-space approach, then Cartesian.
Ctrl+C cancels and holds the arm.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from rcl_interfaces.srv import GetParameters
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

ARM_JOINTS = [
    "gcr16_joint1",
    "gcr16_joint2",
    "gcr16_joint3",
    "gcr16_joint4",
    "gcr16_joint5",
    "gcr16_joint6",
]
MOVEIT_SUCCESS = 1
# Humble MoveItErrorCodes: -21 is FRAME_TRANSFORM_FAILURE, not ROBOT_STATE_STALE (-23).
MOVEIT_ERROR_NAMES = {
    1: "SUCCESS",
    -12: "GOAL_IN_COLLISION",
    -15: "INVALID_GROUP_NAME",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -23: "ROBOT_STATE_STALE",
    -31: "NO_IK_SOLUTION",
}


def moveit_error_name(code):
    """Return the Humble MoveItErrorCodes name for a numeric val."""
    if code is None:
        return "none"
    return MOVEIT_ERROR_NAMES.get(int(code), str(code))


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


def quat_from_rpy(roll, pitch, yaw):
    """Return a geometry_msgs Quaternion from ROS RPY."""
    x, y, z, w = rpy_to_quat_xyzw(roll, pitch, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


def dist3(a, b):
    """Return Euclidean distance between two xyz triples or Point-like objects."""
    ax, ay, az = (a.x, a.y, a.z) if hasattr(a, "x") else a
    bx, by, bz = (b.x, b.y, b.z) if hasattr(b, "x") else b
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def xyz_of(point):
    """Return an (x, y, z) tuple from a Point, Pose, or 3-tuple."""
    if hasattr(point, "position"):
        return (point.position.x, point.position.y, point.position.z)
    if hasattr(point, "x"):
        return (point.x, point.y, point.z)
    return (float(point[0]), float(point[1]), float(point[2]))


def lerp3(a, b, t):
    """Linear interpolate two xyz triples. t=0 is a, t=1 is b."""
    ax, ay, az = a
    bx, by, bz = b
    return (ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t)


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
        self.declare_parameter("approach_duration_sec", 14.0)
        self.declare_parameter("avoid_collisions", False)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.controller_action = self.get_parameter("controller_action").value
        self.max_step = float(self.get_parameter("max_step").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.approach_duration_sec = float(self.get_parameter("approach_duration_sec").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)

        self.pose = pose
        self.corners_world, self.waypoints_world = load_s_world_waypoints(pose)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._static_tf = StaticTransformBroadcaster(self)
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self._param_client = self.create_client(GetParameters, "/move_group/get_parameters")
        self._model_frame = None
        self._published_world_base = False
        self.follow_client = ActionClient(self, FollowJointTrajectory, self.controller_action)
        self._goal_handle = None
        self._stop_requested = False
        self._last_joint_state = None
        self._isaac_joint_state = None
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self._isaac_js_sub = self.create_subscription(
            JointState, "/isaac_joint_states", self._on_isaac_joint_state, 10
        )
        self._exec_event_pub = self.create_publisher(String, "/trajectory_execution_event", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/hawk/s_path", 10)
        self._marker_timer = self.create_timer(0.5, self._publish_markers)
        self._publish_window_tf()
        signal.signal(signal.SIGINT, self._on_stop_signal)
        signal.signal(signal.SIGTERM, self._on_stop_signal)

    def _window_tf_msg(self):
        """Build the world → window_frame static transform from window_frame.json."""
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
        return msg

    def _world_base_tf_msg(self):
        """Build identity world → base_link, matching URDF world_to_base."""
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pose.get("parent_frame", "world")
        msg.child_frame_id = self.base_frame
        msg.transform.rotation.w = 1.0
        return msg

    def _publish_window_tf(self):
        """Broadcast static TFs together so one sendTransform cannot wipe the other."""
        msgs = [self._window_tf_msg()]
        if self._published_world_base:
            msgs.append(self._world_base_tf_msg())
        self._static_tf.sendTransform(msgs)

    def _joint_state_is_ready(self, js):
        """True when a JointState actually contains the GCR arm joints."""
        return js is not None and "gcr16_joint1" in js.name

    def _on_joint_state(self, msg):
        """Cache the latest arm state so a stop can hold the current joints."""
        self._last_joint_state = msg

    def _on_isaac_joint_state(self, msg):
        """Cache Isaac joint states so we know Play is live."""
        self._isaac_joint_state = msg

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
            self.get_logger().warn("Cancel failed: {}".format(exc))
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
            self.get_logger().warn("{} not up; cannot send a hold.".format(self.controller_action))
            return
        names, positions = self._arm_joints_from(js)
        self.execute_joint_positions(names, positions, duration_sec=0.3)

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

    def wait_until_ready(self, timeout_sec=180.0):
        """Block until MoveIt Cartesian IK and Isaac /isaac_joint_states are live."""
        self.get_logger().info(
            "Waiting up to {:.0f}s — PRESS PLAY in Isaac (ROS 2 Bridge on).".format(timeout_sec)
        )
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        last_nudge = 0.0
        while rclpy.ok() and self.get_clock().now() < deadline and not self._stop_requested:
            rclpy.spin_once(self, timeout_sec=0.2)
            cart_ok = self.cartesian_client.service_is_ready()
            isaac_ok = self._joint_state_is_ready(self._isaac_joint_state)
            js_ok = self._joint_state_is_ready(self._last_joint_state)
            if cart_ok and isaac_ok and js_ok:
                self.get_logger().info("MoveIt + Isaac Play are live.")
                self.ensure_world_base_tf()
                self.log_move_group_kinematics()
                self.get_logger().info("planning_frame={}".format(self.planning_frame()))
                return
            now = self.get_clock().now().nanoseconds / 1e9
            if now - last_nudge > 10.0:
                last_nudge = now
                self.get_logger().info(
                    "still waiting: compute_cartesian_path={} isaac_joint_states={} joint_states={}".format(
                        cart_ok, isaac_ok, js_ok
                    )
                )
        if self._stop_requested:
            raise KeyboardInterrupt
        raise RuntimeError(
            "Timed out waiting for Isaac Play + MoveIt. "
            "Play the stage, keep ROS 2 Bridge on, and confirm /isaac_joint_states."
        )

    def log_move_group_kinematics(self):
        """Print the live KDL plugin string from /move_group (null = launch never loaded yaml)."""
        name = "robot_description_kinematics.{}.kinematics_solver".format(self.group_name)
        if not self._param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("No /move_group/get_parameters — cannot read kinematics_solver.")
            return
        req = GetParameters.Request()
        req.names = [name]
        future = self._param_client.call_async(req)
        self.wait_for_future(future, timeout_sec=2.0)
        result = future.result()
        if result is None or not result.values:
            self.get_logger().warn("kinematics_solver param missing: {}".format(name))
            return
        val = result.values[0]
        text = val.string_value if val.string_value else str(val)
        self.get_logger().info("live kinematics_solver={}".format(text))

    def probe_cartesian_here(self, tip):
        """Ask GetCartesianPath for the pose we already have. True if fraction is ~1."""
        response = self.compute_cartesian_path([tip])
        code = response.error_code.val
        self.get_logger().info(
            "Cartesian probe fraction={:.2f} error={} ({})".format(
                response.fraction, code, moveit_error_name(code)
            )
        )
        return response.fraction >= 0.99 and code == MOVEIT_SUCCESS

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
            "No TF from {} to {}. Is duco_arm_example demo.launch.py up?".format(
                self.base_frame, self.ee_frame
            )
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

    def _arm_joints_from(self, js):
        """Pick gcr16_joint1..6 out of a JointState in controller order."""
        index = {name: i for i, name in enumerate(js.name)}
        missing = [name for name in ARM_JOINTS if name not in index]
        if missing:
            raise RuntimeError("JointState missing {}".format(missing))
        return ARM_JOINTS, [js.position[index[name]] for name in ARM_JOINTS]

    def nearest_corner_index(self, tip):
        """Return the S-corner index closest to the current tip."""
        best_i = 0
        best_d = 1e9
        for i, corner in enumerate(self.corners_world):
            d = dist3(tip.position, corner)
            if d < best_d:
                best_i, best_d = i, d
        return best_i, best_d

    def orientation_candidates(self, current):
        """Try current wrist, then tool-Z into the glass (-Y), then a few backups."""
        return [
            ("current", current.orientation),
            ("ee_z_neg_y", quat_from_rpy(math.pi / 2.0, 0.0, 0.0)),
            ("ee_z_pos_y", quat_from_rpy(-math.pi / 2.0, 0.0, 0.0)),
            ("ee_z_neg_x", quat_from_rpy(0.0, math.pi / 2.0, 0.0)),
            ("identity", quat_from_rpy(0.0, 0.0, 0.0)),
        ]

    def _arm_only_js(self, joint_state):
        """Keep only gcr16_joint1..6 so a seed cannot wipe the rest of the robot."""
        names, positions = self._arm_joints_from(joint_state)
        slim = JointState()
        slim.header.stamp = self.get_clock().now().to_msg()
        slim.header.frame_id = self.base_frame
        slim.name = list(names)
        slim.position = [float(p) for p in positions]
        return slim

    def _seed_robot_state(self, joint_state=None):
        """Diff-seed MoveIt with the 6 arm joints. is_diff=False with only 6 joints is incomplete."""
        state = RobotState()
        state.is_diff = True
        seed = joint_state if joint_state is not None else (
            self._last_joint_state or self._isaac_joint_state
        )
        if seed is None:
            return state
        try:
            state.joint_state = self._arm_only_js(seed)
        except RuntimeError:
            state.joint_state = seed
        return state

    def _fresh_start_state(self):
        """Same as a diff seed of the live arm — used by Cartesian and IK."""
        return self._seed_robot_state()

    def planning_frame(self):
        """Return MoveIt's model frame (URDF root). Cartesian must stamp poses in this frame."""
        if self._model_frame:
            return self._model_frame
        fk = self.compute_fk(self.ee_frame)
        if fk is not None and fk.header.frame_id:
            self._model_frame = fk.header.frame_id
            return self._model_frame
        return self.pose.get("parent_frame", "world")

    def ensure_world_base_tf(self):
        """Publish identity world→base_link when that TF is missing (Humble Cartesian needs it)."""
        if self._published_world_base:
            return
        world = self.pose.get("parent_frame", "world")
        deadline = self.get_clock().now() + Duration(seconds=1.0)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                self.tf_buffer.lookup_transform(world, self.base_frame, rclpy.time.Time())
                return
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        self._published_world_base = True
        self._publish_window_tf()
        self.get_logger().warn(
            "No TF {}→{}; publishing identity (matches URDF world_to_base).".format(
                world, self.base_frame
            )
        )

    def compute_fk(self, link_name, joint_state=None):
        """Call MoveIt GetPositionFK. Pose is stamped in the model frame."""
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("/compute_fk is missing.")
            return None
        request = GetPositionFK.Request()
        request.fk_link_names = [link_name]
        request.robot_state = self._seed_robot_state(joint_state)
        future = self.fk_client.call_async(request)
        self.wait_for_future(future, timeout_sec=2.0)
        if self._stop_requested:
            raise KeyboardInterrupt
        response = future.result()
        if response is None or response.error_code.val != MOVEIT_SUCCESS:
            code = None if response is None else response.error_code.val
            self.get_logger().warn(
                "GetPositionFK failed code={} ({})".format(code, moveit_error_name(code))
            )
            return None
        if not response.pose_stamped:
            return None
        return response.pose_stamped[0]

    def lookup_tip_in_planning_frame(self):
        """Prefer MoveIt FK (model frame) so IK/Cartesian poses match the request header."""
        fk = self.compute_fk(self.ee_frame)
        if fk is not None:
            if fk.header.frame_id:
                self._model_frame = fk.header.frame_id
            return fk.pose
        return self.lookup_current_tip_pose()

    def compute_ik(self, xyz, orientation, seed_js=None, set_link_name=True, frame_id=None):
        """Call MoveIt GetPositionIK for one pose. Return JointState or None."""
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/compute_ik is missing. Is move_group running?")
        pose = PoseStamped()
        pose.header.frame_id = frame_id or self.planning_frame()
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        pose.pose.orientation = orientation
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.group_name
        if set_link_name:
            request.ik_request.ik_link_name = self.ee_frame
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 2
        request.ik_request.timeout.nanosec = 0
        request.ik_request.robot_state = self._seed_robot_state(seed_js)
        future = self.ik_client.call_async(request)
        self.wait_for_future(future, timeout_sec=4.0)
        if self._stop_requested:
            raise KeyboardInterrupt
        response = future.result()
        if response is None:
            self._last_ik_code = None
            return None
        if response.error_code.val != MOVEIT_SUCCESS:
            self._last_ik_code = response.error_code.val
            return None
        self._last_ik_code = MOVEIT_SUCCESS
        return response.solution.joint_state

    def compute_ik_with_retries(self, xyz, orientation, seed_js=None, attempts=3):
        """IK in the planning frame with the group tip link. Retries keep a stable request."""
        del attempts
        for set_link in (True, False):
            js = self.compute_ik(
                xyz, orientation, seed_js=seed_js, set_link_name=set_link
            )
            if js is not None:
                return js
        return None

    def probe_ik_at_current_tip(self, tip):
        """IK to MoveIt's FK pose. Failure at all-zero stretch is a KDL singularity, not a dead plugin."""
        xyz = xyz_of(tip)
        q = tip.orientation
        self.get_logger().info(
            "IK probe in {} xyz=({:.3f},{:.3f},{:.3f}) quat=({:.3f},{:.3f},{:.3f},{:.3f})".format(
                self.planning_frame(),
                xyz[0],
                xyz[1],
                xyz[2],
                q.x,
                q.y,
                q.z,
                q.w,
            )
        )
        js = self.compute_ik_with_retries(xyz, q, seed_js=None, attempts=1)
        if js is not None:
            self.get_logger().info("IK probe OK — solver is alive.")
            return True
        seed = self._last_joint_state or self._isaac_joint_state
        if seed is not None:
            try:
                names, pos = self._arm_joints_from(seed)
                self.get_logger().info(
                    "seed joints {} = {}".format(
                        names, ["{:.3f}".format(p) for p in pos]
                    )
                )
            except RuntimeError as exc:
                self.get_logger().warn(str(exc))
        code = getattr(self, "_last_ik_code", None)
        self.get_logger().warn(
            "IK probe FAILED code={} ({}). Stretched all-zero is a KDL singularity; "
            "will joint-space jog off the pole if needed.".format(
                code, moveit_error_name(code)
            )
        )
        return False

    def arm_near_zero(self, eps=0.08):
        """True when gcr16_joint1..6 are all near zero (fully stretched GCR home)."""
        seed = self._last_joint_state or self._isaac_joint_state
        if seed is None:
            return False
        try:
            _, pos = self._arm_joints_from(seed)
        except RuntimeError:
            return False
        return max(abs(p) for p in pos) < eps

    def jog_off_singularity(self):
        """Fold j2/j3 a little in joint space so KDL can leave the stretched pole."""
        seed = self._last_joint_state or self._isaac_joint_state
        if seed is None:
            return False
        names, pos = self._arm_joints_from(seed)
        target = list(pos)
        target[1] = pos[1] + 0.40
        target[2] = pos[2] + 0.40
        self.get_logger().info(
            "All-zero stretch is a KDL singularity — joint-space jog j2/j3 +0.40 rad."
        )
        self.execute_joint_positions(names, target, duration_sec=4.0)
        rclpy.spin_once(self, timeout_sec=0.2)
        return True

    def cartesian_nudge(self, xyz, orientation):
        """One Cartesian waypoint from the current state. Return a RobotTrajectory or None."""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation = orientation
        response = self.compute_cartesian_path([pose])
        self._last_ik_code = response.error_code.val
        if response.fraction < 0.95:
            return None
        return response.solution

    def execute_joint_positions(self, names, positions, duration_sec):
        """Send a 2-point joint-space move from the current arm state to positions."""
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while self._last_joint_state is None and rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._last_joint_state is None:
            raise RuntimeError("No /joint_states for the approach move.")
        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("{} is missing.".format(self.controller_action))
        start_names, start_pos = self._arm_joints_from(self._last_joint_state)
        if list(start_names) != list(names):
            idx = {n: i for i, n in enumerate(names)}
            positions = [positions[idx[n]] for n in start_names]
            names = start_names
        traj = JointTrajectory()
        traj.joint_names = list(names)
        start = JointTrajectoryPoint()
        start.positions = list(start_pos)
        start.time_from_start.sec = 0
        start.time_from_start.nanosec = 0
        goal = JointTrajectoryPoint()
        goal.positions = [float(p) for p in positions]
        t = max(0.2, float(duration_sec))
        goal.time_from_start.sec = int(t)
        goal.time_from_start.nanosec = int((t - int(t)) * 1e9)
        traj.points = [start, goal]
        action_goal = FollowJointTrajectory.Goal()
        action_goal.trajectory = traj
        self.get_logger().info(
            "Joint approach {} pts over {:.1f}s to {}".format(
                len(traj.points), t, self.controller_action
            )
        )
        send_future = self.follow_client.send_goal_async(action_goal)
        self.wait_for_future(send_future, timeout_sec=10.0)
        if self._stop_requested:
            raise KeyboardInterrupt
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Approach FollowJointTrajectory was rejected.")
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self.wait_for_future(result_future, timeout_sec=t + 5.0)
        if self._stop_requested:
            self.stop_motion_now()
            raise KeyboardInterrupt
        self._goal_handle = None
        return result_future.result().result

    def execute_joint_path(self, names, points, duration_sec):
        """Stream a multi-point joint-space path (seeded IK walk) to the controller."""
        if not points:
            raise RuntimeError("Empty joint path.")
        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("{} is missing.".format(self.controller_action))
        start_names, start_pos = self._arm_joints_from(self._last_joint_state)
        traj = JointTrajectory()
        traj.joint_names = list(start_names)
        count = len(points)
        t_total = max(1.0, float(duration_sec))
        start = JointTrajectoryPoint()
        start.positions = list(start_pos)
        start.time_from_start.sec = 0
        traj.points.append(start)
        for i, pos in enumerate(points):
            if list(names) != list(start_names):
                idx = {n: j for j, n in enumerate(names)}
                pos = [pos[idx[n]] for n in start_names]
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in pos]
            t = t_total * (i + 1) / float(count)
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(point)
        action_goal = FollowJointTrajectory.Goal()
        action_goal.trajectory = traj
        self.get_logger().info(
            "IK-walk execute {} pts over {:.1f}s".format(len(traj.points), t_total)
        )
        send_future = self.follow_client.send_goal_async(action_goal)
        self.wait_for_future(send_future, timeout_sec=10.0)
        if self._stop_requested:
            raise KeyboardInterrupt
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Approach FollowJointTrajectory was rejected.")
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self.wait_for_future(result_future, timeout_sec=t_total + 5.0)
        if self._stop_requested:
            self.stop_motion_now()
            raise KeyboardInterrupt
        self._goal_handle = None
        return result_future.result().result

    def walk_ik_to(self, target_xyz, orientation, step_m=0.07):
        """Walk the tip in small steps: IK if it works, else a short Cartesian nudge."""
        tip = self.lookup_tip_in_planning_frame()
        start = xyz_of(tip)
        gap = dist3(start, target_xyz)
        steps = max(1, int(math.ceil(gap / step_m)))
        self.get_logger().info(
            "IK walk {} steps, {:.3f}m, keep current wrist.".format(steps, gap)
        )
        seed = self._last_joint_state
        path = []
        names = ARM_JOINTS
        for i in range(1, steps + 1):
            xyz = lerp3(start, target_xyz, i / float(steps))
            js = self.compute_ik_with_retries(xyz, orientation, seed_js=seed, attempts=1)
            if js is not None:
                names, pos = self._arm_joints_from(js)
                path.append(pos)
                seed = js
                continue
            if path:
                duration = max(2.0, min(12.0, len(path) * 0.35))
                self.execute_joint_path(names, path, duration)
                path = []
            cart = self.cartesian_nudge(xyz, orientation)
            if cart is None:
                code = getattr(self, "_last_ik_code", None)
                self.get_logger().warn(
                    "IK+Cartesian died at step {}/{} xyz=({:.3f},{:.3f},{:.3f}) code={} ({})".format(
                        i,
                        steps,
                        xyz[0],
                        xyz[1],
                        xyz[2],
                        code,
                        moveit_error_name(code),
                    )
                )
                return False
            self.get_logger().info("Cartesian nudge step {}/{}".format(i, steps))
            saved = self.duration_sec
            self.duration_sec = 1.2
            try:
                self.execute_on_arm_controller(cart)
            finally:
                self.duration_sec = saved
            seed = self._last_joint_state
        if path:
            duration = max(4.0, min(18.0, len(path) * 0.45))
            self.execute_joint_path(names, path, duration)
        return True

    def approach_first_corner(self, tip):
        """Walk to the nearest S corner, keeping the current wrist. Return orientation."""
        del tip
        self.ensure_world_base_tf()
        tip = self.lookup_tip_in_planning_frame()
        ik_ok = self.probe_ik_at_current_tip(tip)
        cart_ok = self.probe_cartesian_here(tip)
        if (not ik_ok or not cart_ok) and self.arm_near_zero():
            self.jog_off_singularity()
            tip = self.lookup_tip_in_planning_frame()
            ik_ok = self.probe_ik_at_current_tip(tip)
            cart_ok = self.probe_cartesian_here(tip)
        if not ik_ok and not cart_ok:
            self.get_logger().warn(
                "Probes still failing after jog; trying the IK walk anyway."
            )
        elif not ik_ok:
            self.get_logger().warn(
                "GetPositionIK is weak at this pose; approach will prefer Cartesian nudges."
            )
        corner_i, gap = self.nearest_corner_index(tip)
        xyz = self.corners_world[corner_i]
        self.get_logger().info(
            "Approach corner {} at ({:.3f},{:.3f},{:.3f}), {:.3f}m from tip.".format(
                corner_i, xyz[0], xyz[1], xyz[2], gap
            )
        )
        orientation = tip.orientation
        if not self.walk_ik_to(xyz, orientation):
            order = sorted(
                range(len(self.corners_world)),
                key=lambda i: -self.corners_world[i][2],
            )
            hit = False
            for idx in order:
                if idx == corner_i:
                    continue
                alt = self.corners_world[idx]
                self.get_logger().info(
                    "Retry walk to corner {} ({:.3f},{:.3f},{:.3f})".format(
                        idx, alt[0], alt[1], alt[2]
                    )
                )
                if self.walk_ik_to(alt, orientation):
                    corner_i = idx
                    hit = True
                    break
            if not hit:
                raise RuntimeError(
                    "No IK/Cartesian walk to the S pane from home. "
                    "Confirm RViz can Cartesian-plan a tiny move first."
                )
        ordered = self.corners_world[corner_i:] + self.corners_world[:corner_i]
        spacing = float(self.pose.get("spacing_m", 0.03))
        self.waypoints_world = densify_closed(ordered, spacing)
        return orientation

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
        request.header.frame_id = self.planning_frame()
        request.header.stamp = rclpy.time.Time().to_msg()
        request.start_state = self._seed_robot_state()
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
                "{} is missing. Is duco_arm_controller spawned?".format(self.controller_action)
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        self.get_logger().info(
            "Sending {} pts over {:.1f}s to {}".format(
                len(joint_traj.points), self.duration_sec, self.controller_action
            )
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
    """Wait for Isaac Play, approach the S pane, then Cartesian-trace it."""
    parser = argparse.ArgumentParser(description="GCR joint6 tip follows S_path.")
    parser.add_argument("--stop", action="store_true", help="Halt leftover motion only.")
    parser.add_argument("--preview", action="store_true", help="Publish RViz markers, no motion.")
    parser.add_argument(
        "--no-approach",
        action="store_true",
        help="Skip joint-space IK to the pane (old behavior: Cartesian from here).",
    )
    parser.add_argument("--pose", default="", help="Override window_frame.json")
    args, ros_args = parser.parse_known_args()

    pose = load_window_pose(args.pose or None)
    rclpy.init(args=ros_args)
    node = SPathFollower(pose)
    cmin = [min(p[i] for p in node.corners_world) for i in range(3)]
    cmax = [max(p[i] for p in node.corners_world) for i in range(3)]
    # Humble's Python 3.10 chokes on f"{list[i]:.3f}" inside literal [].
    node.get_logger().info(
        "S world x[{:.3f},{:.3f}] y[{:.3f},{:.3f}] z[{:.3f},{:.3f}] n={} ee={} group={}".format(
            cmin[0],
            cmax[0],
            cmin[1],
            cmax[1],
            cmin[2],
            cmax[2],
            len(node.waypoints_world),
            node.ee_frame,
            node.group_name,
        )
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
        node.wait_until_ready()
        tip = node.lookup_current_tip_pose()
        node.get_logger().info(
            "Tip {} in {}: x={:.3f} y={:.3f} z={:.3f}".format(
                node.ee_frame,
                node.base_frame,
                tip.position.x,
                tip.position.y,
                tip.position.z,
            )
        )
        if args.no_approach:
            orientation = tip.orientation
            node.get_logger().info("No approach — Cartesian from the current pose.")
        else:
            orientation = node.approach_first_corner(tip)
        waypoints = node.sample_path_waypoints(orientation)
        cartesian = node.compute_cartesian_path(waypoints)
        node.get_logger().info(
            "Cartesian fraction={:.2f} error={} ({})".format(
                cartesian.fraction,
                cartesian.error_code.val,
                moveit_error_name(cartesian.error_code.val),
            )
        )
        if cartesian.fraction < 0.99:
            node.get_logger().error(
                "MoveIt could not cover the whole S loop. Edit "
                "window_path/window_frame.json translation_m and re-run."
            )
            return
        result = node.execute_on_arm_controller(cartesian.solution)
        node.get_logger().info("FollowJointTrajectory error_code={}".format(result.error_code))
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
