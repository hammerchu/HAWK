"""Draw S (green) and L (gold) rectangles in the Isaac Sim viewport via debug_draw.

Run this *inside* Isaac (Window → Script Editor → Open this file → Run).
ROS python3 cannot import isaacsim.util.debug_draw.

Polylines use draw_lines (not spline). Dots are waypoints; V chevrons show
travel direction; the white dot is the loop start. Re-run to refresh.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

S_COLOR = (0.15, 0.95, 0.35, 1.0)
L_COLOR = (0.95, 0.75, 0.15, 1.0)
START_COLOR = (1.0, 1.0, 1.0, 1.0)
LINE_WIDTH = 5.0
ARROW_WIDTH = 3.0
ARROW_LEN = 0.08
WAYPOINT_SPACING = 0.12
CORNER_PT_SIZE = 16.0
WP_PT_SIZE = 8.0
START_PT_SIZE = 22.0
WINDOW_PRIM_PATHS = ("/window_frame", "/World/window_frame")


def _generated_dir():
    """Return window_path/generated, including Script Editor and sim-box paths."""
    candidates = []
    if "__file__" in globals():
        candidates.append(Path(__file__).resolve().parent / "generated")
    candidates.append(Path.home() / "DEV/HAWK/isaac_sim_test/window_path/generated")
    for path in candidates:
        if (path / "S_path.json").is_file():
            return path
    raise FileNotFoundError(
        "No S_path.json. Run export_aligned.py, then re-run this in Isaac."
    )


def _load_path_json(generated, name):
    """Load one exported path JSON (S_path.json or L_path.json)."""
    payload = json.loads((generated / "{}_path.json".format(name)).read_text())
    return payload["path"]


def _as_xyz(points):
    """Return a list of (x, y, z) float tuples, duplicating the first to close."""
    xyz = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
    if not xyz:
        raise ValueError("empty path")
    if xyz[0] != xyz[-1]:
        xyz.append(xyz[0])
    return xyz


def _draw_polyline(draw, points, color, width):
    """Stroke a closed polyline as straight segments (DXF LWPOLYLINE, not a spline)."""
    starts = points[:-1]
    ends = points[1:]
    colors = [color] * len(starts)
    widths = [float(width)] * len(starts)
    draw.draw_lines(starts, ends, colors, widths)


def _vsub(a, b):
    """Return vector a - b."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vadd(a, b):
    """Return vector a + b."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vmul(a, s):
    """Return vector a scaled by s."""
    return (a[0] * s, a[1] * s, a[2] * s)


def _vlen(a):
    """Return Euclidean length of a 3-vector."""
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _vnorm(a):
    """Return a unit vector, or (1, 0, 0) if a is degenerate."""
    n = _vlen(a)
    if n < 1e-9:
        return (1.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def _in_plane_perp(direction):
    """Return a unit perpendicular in the path plane (prefer world-Z cross)."""
    dx, dy, dz = direction
    perp = (-dy, dx, 0.0)
    if _vlen(perp) < 1e-9:
        perp = (-dz, 0.0, dx)
    return _vnorm(perp)


def _edge_waypoints(closed_pts, spacing):
    """Sample points along each edge, skipping the duplicated closing vertex."""
    samples = []
    for start, end in zip(closed_pts[:-1], closed_pts[1:]):
        delta = _vsub(end, start)
        length = _vlen(delta)
        count = max(1, int(math.floor(length / spacing)))
        for i in range(count):
            t = i / float(count)
            samples.append(_vadd(start, _vmul(delta, t)))
    return samples


def _arrow_segments(start, end, size):
    """Return (starts, ends) for a V chevron pointing start → end, near 72% of the edge."""
    delta = _vsub(end, start)
    length = _vlen(delta)
    if length < size * 2.0:
        return [], []
    direction = _vnorm(delta)
    tip = _vadd(start, _vmul(delta, 0.72))
    back = _vadd(tip, _vmul(direction, -size))
    side = _vmul(_in_plane_perp(direction), size * 0.45)
    left = _vadd(back, side)
    right = _vsub(back, side)
    return [tip, tip, tip], [left, right, back]


def _draw_waypoints_and_arrows(draw, closed_pts, color):
    """Draw edge samples, a bigger start dot, and direction chevrons on each side."""
    corners = closed_pts[:-1]
    samples = _edge_waypoints(closed_pts, WAYPOINT_SPACING)
    if hasattr(draw, "draw_points") and samples:
        draw.draw_points(samples, [color] * len(samples), [WP_PT_SIZE] * len(samples))
        draw.draw_points(corners, [color] * len(corners), [CORNER_PT_SIZE] * len(corners))
        draw.draw_points([closed_pts[0]], [START_COLOR], [START_PT_SIZE])
    arrow_starts = []
    arrow_ends = []
    for start, end in zip(closed_pts[:-1], closed_pts[1:]):
        heads, tails = _arrow_segments(start, end, ARROW_LEN)
        arrow_starts.extend(heads)
        arrow_ends.extend(tails)
    if arrow_starts:
        colors = [color] * len(arrow_starts)
        widths = [ARROW_WIDTH] * len(arrow_starts)
        draw.draw_lines(arrow_starts, arrow_ends, colors, widths)


def _find_window_prim(stage):
    """Return the window_frame prim if it is on the stage."""
    for path in WINDOW_PRIM_PATHS:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            return prim
    for prim in stage.Traverse():
        if prim.GetName() == "window_frame" and prim.IsValid():
            return prim
    return None


def _cad_to_stage_world(cad_points, prim):
    """Transform CAD-local ROS metres through the live window_frame world xform."""
    from pxr import Gf, Usd, UsdGeom

    xform = UsdGeom.Xformable(prim)
    world = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    out = []
    for x, y, z in cad_points:
        p = world.Transform(Gf.Vec3d(float(x), float(y), float(z)))
        out.append((float(p[0]), float(p[1]), float(p[2])))
    return out


def _points_for_path(path, window_prim):
    """Prefer live Isaac xform so a dragged pane still matches the stroke."""
    cad = path.get("corners_cad_m")
    if window_prim is not None and cad:
        return _as_xyz(_cad_to_stage_world(cad, window_prim))
    world = path.get("corners_world_m") or path.get("waypoints_world_m")
    return _as_xyz(world)


def _acquire_draw():
    """Return the Isaac debug-draw interface (4.5+ name, then older omni.isaac)."""
    try:
        import omni.kit.app

        manager = omni.kit.app.get_app().get_extension_manager()
        for ext in ("isaacsim.util.debug_draw", "omni.isaac.debug_draw"):
            if manager.is_extension_enabled(ext):
                continue
            try:
                manager.set_extension_enabled_immediate(ext, True)
            except Exception:
                pass
    except Exception:
        pass
    try:
        from isaacsim.util.debug_draw import _debug_draw
    except ImportError:
        from omni.isaac.debug_draw import _debug_draw
    return _debug_draw.acquire_debug_draw_interface()


def draw_s_l_paths():
    """Clear previous strokes and draw S (green) plus L (gold) in the viewport."""
    draw = _acquire_draw()
    draw.clear_lines()
    if hasattr(draw, "clear_points"):
        draw.clear_points()

    generated = _generated_dir()
    window_prim = None
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            window_prim = _find_window_prim(stage)
    except Exception:
        window_prim = None

    s_pts = _points_for_path(_load_path_json(generated, "S"), window_prim)
    l_pts = _points_for_path(_load_path_json(generated, "L"), window_prim)
    _draw_polyline(draw, s_pts, S_COLOR, LINE_WIDTH)
    _draw_polyline(draw, l_pts, L_COLOR, LINE_WIDTH)
    _draw_waypoints_and_arrows(draw, s_pts, S_COLOR)
    _draw_waypoints_and_arrows(draw, l_pts, L_COLOR)

    source = "live /window_frame xform" if window_prim else "JSON world corners"
    print("Drew S (green, {} pts) and L (gold, {} pts) from {}.".format(
        len(s_pts), len(l_pts), source
    ))
    print("Strokes persist until you re-run this script or call draw.clear_lines().")


# Isaac Script Editor Run executes this file (often not as __main__). Always draw.
draw_s_l_paths()
