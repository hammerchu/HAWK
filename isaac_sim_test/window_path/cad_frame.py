#!/usr/bin/env python3
"""One law for window mesh and DXF: Fusion cm Y-up -> ROS metres Z-up + parent pose."""

from __future__ import annotations

import json
import math
from pathlib import Path

# DXF $INSUNITS=5 and STEP SI_UNIT centimetre. FBX UnitScaleFactor=1 (cm).
CAD_CM_TO_M = 0.01

# Identity quat for USD (w, x, y, z) and ROS (x, y, z, w).
_RX90_WXYZ = (
    0.7071067811865476,
    0.7071067811865476,
    0.0,
    0.0,
)


def fusion_to_ros(x, y, z):
    """Map one Fusion Y-up point to ROS Z-up. Same matrix as GCR mesh export."""
    return (x, -z, y)


def cad_cm_to_ros_m(x_cm, y_cm, z_cm):
    """Scale CAD centimetres to metres, then Fusion Y-up to ROS Z-up."""
    x, y, z = fusion_to_ros(x_cm, y_cm, z_cm)
    return (x * CAD_CM_TO_M, y * CAD_CM_TO_M, z * CAD_CM_TO_M)


def rpy_to_matrix(roll, pitch, yaw):
    """Return a 3x3 rotation for ROS RPY (fixed X then Y then Z)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotate_translate(point, matrix, origin):
    """Apply R * point + origin to a 3-tuple."""
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + origin[0],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + origin[1],
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + origin[2],
    )


def apply_window_pose(points_cad, translation_m, rpy_rad, approach_offset_m=0.0):
    """Move CAD-local ROS points into world. Offset is along CAD +Y (robot side)."""
    shifted = []
    for x, y, z in points_cad:
        shifted.append((x, y + approach_offset_m, z))
    matrix = rpy_to_matrix(*rpy_rad)
    return [rotate_translate(p, matrix, translation_m) for p in shifted]


def rpy_to_quat_xyzw(roll, pitch, yaw):
    """Return ROS xyzw quaternion for the given RPY."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose_file_path():
    """Return the default window_frame.json next to this package."""
    return Path(__file__).resolve().parent / "window_frame.json"


def load_window_pose(path=None):
    """Load the single parent pose applied to both window mesh and path."""
    pose_path = Path(path) if path else pose_file_path()
    data = json.loads(pose_path.read_text(encoding="utf-8"))
    translation = tuple(float(v) for v in data["translation_m"])
    rpy = tuple(float(v) for v in data["rpy_rad"])
    if len(translation) != 3 or len(rpy) != 3:
        raise ValueError(f"{pose_path} needs translation_m[3] and rpy_rad[3]")
    data["translation_m"] = translation
    data["rpy_rad"] = rpy
    data["approach_offset_m"] = float(data.get("approach_offset_m", 0.0))
    data["_path"] = pose_path
    return data


def fbx_child_orient_wxyz():
    """USD (w,x,y,z) for +90 deg about X: Fusion (x,y,z) -> ROS (x,-z,y)."""
    return _RX90_WXYZ
