"""Shared CAD frame for the window mesh and DXF tool paths."""

from .cad_frame import (
    apply_window_pose,
    cad_cm_to_ros_m,
    fusion_to_ros,
    load_window_pose,
)
from .dxf_path import densify_closed, load_s_rectangle_cm
