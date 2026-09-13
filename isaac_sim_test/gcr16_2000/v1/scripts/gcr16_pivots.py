#!/usr/bin/env python3
"""Measured GCR16 pins (ROS Z-up, meters) from Fusion jointN_root cylinders."""

# World pins after Fusion Y-up -> ROS (x, -z, y). Axes are unit vectors.
PIVOTS = [
    {
        "name": "base_root",
        "joint": "world_to_base",
        "pin": (0.0, 0.0, 0.002),
        "axis": (0.0, 0.0, -1.0),
    },
    {
        "name": "joint1_root",
        "joint": "gcr16_joint1",
        "pin": (0.0, 0.0, 0.1176),
        "axis": (0.0, 0.0, 1.0),
    },
    {
        "name": "joint2_root",
        "joint": "gcr16_joint2",
        "pin": (0.1239, 0.0, 0.235),
        "axis": (1.0, 0.0, 0.0),
    },
    {
        "name": "joint3_root",
        "joint": "gcr16_joint3",
        "pin": (0.133301, 0.0, 1.2125),
        "axis": (-1.0, 0.0, 0.0),
    },
    {
        "name": "joint4_root",
        "joint": "gcr16_joint4",
        "pin": (0.103301, 0.0, 2.109),
        "axis": (1.0, 0.0, 0.0),
    },
    {
        "name": "joint5_root",
        "joint": "gcr16_joint5",
        "pin": (0.160801, 0.0, 2.1775),
        "axis": (0.0, 0.0, 1.0),
    },
    {
        "name": "joint6_root",
        "joint": "gcr16_joint6",
        "pin": (0.229301, 0.0, 2.235),
        "axis": (1.0, 0.0, 0.0),
    },
]

# Distinct RGB so you can match a slider to a tube in RViz.
PIVOT_COLORS = [
    (0.6, 0.6, 0.6),
    (1.0, 0.2, 0.2),
    (1.0, 0.7, 0.1),
    (0.2, 0.8, 0.2),
    (0.2, 0.6, 1.0),
    (0.7, 0.3, 1.0),
    (1.0, 0.3, 0.7),
]


def format_pivot_table():
    """Return a printable table of measured pins and axes."""
    lines = ["[gcr16 pivot] measured ROS Z-up pins (m) — arrows in RViz on /gcr16/pivot_markers"]
    for row in PIVOTS:
        pin = " ".join(f"{c:.4f}" for c in row["pin"])
        axis = " ".join(f"{c:.3f}" for c in row["axis"])
        lines.append(f"  {row['name']:<14} {row['joint']:<16} pin {pin:<24} axis {axis}")
    lines.append(
        "[gcr16 pivot] arrow on the CAD hinge + TF on the arrow = URDF ok; "
        "arrow on hinge but mesh rips = mesh frame; arrow in empty space = pin remap"
    )
    return "\n".join(lines)
