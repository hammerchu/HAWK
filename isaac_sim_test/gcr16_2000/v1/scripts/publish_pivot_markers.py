#!/usr/bin/env python3
"""Publish RViz markers for measured GCR16 joint pins and spin axes."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from gcr16_pivots import PIVOT_COLORS, PIVOTS, format_pivot_table


def _quat_from_axis(axis, fallback=(1.0, 0.0, 0.0)):
    """Return a quaternion that rotates +X onto the given axis."""
    ax, ay, az = axis
    length = math.sqrt(ax * ax + ay * ay + az * az)
    if length < 1e-9:
        ax, ay, az = fallback
        length = 1.0
    ax, ay, az = ax / length, ay / length, az / length
    # +X cross dest
    cx, cy, cz = 0.0 * az - 0.0 * ay, 0.0 * ax - 1.0 * az, 1.0 * ay - 0.0 * ax
    dot = 1.0 * ax
    if dot > 0.999999:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    if dot < -0.999999:
        return Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
    cr = math.sqrt(cx * cx + cy * cy + cz * cz)
    ang = math.atan2(cr, dot)
    s = math.sin(ang * 0.5) / cr
    return Quaternion(x=cx * s, y=cy * s, z=cz * s, w=math.cos(ang * 0.5))


def _color(rgb, alpha=0.95):
    """Build a ColorRGBA from an (r, g, b) tuple."""
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=alpha)


class PivotMarkerNode(Node):
    """Publish spheres, axis arrows, and labels for each measured pin."""

    def __init__(self):
        """Create the /gcr16/pivot_markers publisher."""
        super().__init__("gcr16_pivot_markers")
        self._pub = self.create_publisher(MarkerArray, "/gcr16/pivot_markers", 10)
        self._timer = self.create_timer(0.5, self._publish)
        self.get_logger().info("\n" + format_pivot_table())

    def _publish(self):
        """Publish the full MarkerArray in base_link (home-pose world)."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        markers = []
        for index, row in enumerate(PIVOTS):
            rgb = PIVOT_COLORS[index % len(PIVOT_COLORS)]
            pin = row["pin"]
            axis = row["axis"]
            markers.append(self._sphere(header, index, pin, rgb))
            markers.append(self._arrow(header, index, pin, axis, rgb))
            markers.append(self._label(header, index, pin, row["joint"]))
        self._pub.publish(MarkerArray(markers=markers))

    def _sphere(self, header, index, pin, rgb):
        """Pin location as a small sphere."""
        m = Marker()
        m.header = header
        m.ns = "pivot_pin"
        m.id = index
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(x=pin[0], y=pin[1], z=pin[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color = _color(rgb)
        return m

    def _arrow(self, header, index, pin, axis, rgb):
        """Spin axis as an arrow centered on the pin (length 0.22 m)."""
        half = 0.11
        m = Marker()
        m.header = header
        m.ns = "pivot_axis"
        m.id = index
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position = Point(
            x=pin[0] - axis[0] * half,
            y=pin[1] - axis[1] * half,
            z=pin[2] - axis[2] * half,
        )
        m.pose.orientation = _quat_from_axis(axis)
        m.scale.x = 0.22
        m.scale.y = 0.012
        m.scale.z = 0.012
        m.color = _color(rgb)
        return m

    def _label(self, header, index, pin, text):
        """Joint name floating next to the pin."""
        m = Marker()
        m.header = header
        m.ns = "pivot_label"
        m.id = index
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position = Point(x=pin[0] + 0.06, y=pin[1] + 0.06, z=pin[2] + 0.06)
        m.pose.orientation.w = 1.0
        m.scale.z = 0.05
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = text
        return m


def main(argv=None):
    """Start the pivot-marker node until Ctrl+C."""
    scripts = Path(__file__).resolve().parent
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    rclpy.init(args=argv)
    node = PivotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
