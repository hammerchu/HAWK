"""Draw S (green) and L (gold) rectangles in the Isaac Sim viewport via debug_draw.

Run this *inside* Isaac (Window → Script Editor → Open this file → Run).
ROS python3 cannot import isaacsim.util.debug_draw.

Use draw_lines (straight segments), not draw_lines_spline — a spline through
four corners becomes an oval. Re-run to refresh; the script clears old lines.
"""

from __future__ import annotations

import json
from pathlib import Path

S_COLOR = (0.15, 0.95, 0.35, 1.0)
L_COLOR = (0.95, 0.75, 0.15, 1.0)
LINE_WIDTH = 5.0
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

    source = "live /window_frame xform" if window_prim else "JSON world corners"
    print("Drew S (green, {} pts) and L (gold, {} pts) from {}.".format(
        len(s_pts), len(l_pts), source
    ))
    print("Strokes persist until you re-run this script or call draw.clear_lines().")


# Isaac Script Editor Run executes this file (often not as __main__). Always draw.
draw_s_l_paths()
