#!/usr/bin/env python3
"""Read Fusion DXF LWPOLYLINE paths. S is rebuilt as the unique-corner rectangle."""

from __future__ import annotations

import math
from pathlib import Path


def default_dxf_dir():
    """Return isaac_sim_test/path next to this package."""
    return Path(__file__).resolve().parent.parent / "path"


def parse_lwpolylines(dxf_path):
    """Parse LWPOLYLINE vertices from an ASCII DXF. Returns list of point lists."""
    lines = Path(dxf_path).read_text(encoding="utf-8", errors="replace").splitlines()
    polys = []
    i = 0
    while i < len(lines):
        if lines[i].strip() != "LWPOLYLINE":
            i += 1
            continue
        pts = []
        x_hold = None
        z_hold = 0.0
        i += 1
        while i < len(lines):
            code = lines[i].strip()
            val = lines[i + 1].strip() if i + 1 < len(lines) else ""
            if code == "0":
                break
            if code == "10":
                x_hold = float(val)
            elif code == "20" and x_hold is not None:
                pts.append((x_hold, float(val), z_hold))
                x_hold = None
            elif code == "30":
                z_hold = float(val)
                if pts:
                    px, py, _ = pts[-1]
                    pts[-1] = (px, py, z_hold)
            i += 2
        if pts:
            polys.append(pts)
    return polys


def unique_xy_corners(points, decimals=4):
    """Return unique (x, y, z) corners rounded so Fusion noise collapses."""
    seen = []
    keys = set()
    for x, y, z in points:
        key = (round(x, decimals), round(y, decimals))
        if key in keys:
            continue
        keys.add(key)
        seen.append((x, y, z))
    return seen


def rectangle_from_points(points):
    """Build a closed-ready 4-corner rectangle from DXF XY extents (CAD cm)."""
    if not points:
        raise ValueError("No DXF points to build a rectangle")
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    z = points[0][2]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    # BL -> BR -> TR -> TL, matching the S intercept loop.
    return [
        (xmin, ymin, z),
        (xmax, ymin, z),
        (xmax, ymax, z),
        (xmin, ymax, z),
    ]


def load_s_rectangle_cm(dxf_path=None):
    """Load S_path.dxf and return the glass-intercept rectangle in CAD cm."""
    path = Path(dxf_path) if dxf_path else default_dxf_dir() / "S_path.dxf"
    polys = parse_lwpolylines(path)
    flat = [pt for poly in polys for pt in poly]
    return rectangle_from_points(unique_xy_corners(flat))


def load_l_rectangle_cm(dxf_path=None):
    """Load L_path.dxf and return its outer rectangle in CAD cm."""
    path = Path(dxf_path) if dxf_path else default_dxf_dir() / "L_path.dxf"
    polys = parse_lwpolylines(path)
    flat = [pt for poly in polys for pt in poly]
    return rectangle_from_points(unique_xy_corners(flat))


def densify_closed(corners, spacing_m):
    """Insert points along each edge so spacing is at most spacing_m. Closes the loop."""
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be > 0")
    if len(corners) < 2:
        return list(corners)
    out = []
    count = len(corners)
    for i in range(count):
        ax, ay, az = corners[i]
        bx, by, bz = corners[(i + 1) % count]
        dx, dy, dz = bx - ax, by - ay, bz - az
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        steps = max(1, int(math.ceil(length / spacing_m)))
        for s in range(steps):
            t = s / float(steps)
            out.append((ax + dx * t, ay + dy * t, az + dz * t))
    out.append(corners[0])
    return out
