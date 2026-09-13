#!/usr/bin/env python3
"""Tessellate per-link Fusion STEPs into link-local ROS STLs.

Do not pass STEP paths as FreeCADCmd arguments. Use GCR16_LINK_DIR and GCR16_OUT.

Pipeline per GCR16-Jn.step:
  1) Part.read
  2) Keep raw or assembly placement — whichever centroid is closer to the
     Fusion instance origin
  3) Fusion Y-up (x, y, z) -> ROS Z-up (x, -z, y)
  4) Subtract that link's measured ROS pin so the STL lives in the URDF
     link frame (visual origin 0 0 0)
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _env_paths():
    """Read link-STEP dir and mesh output dir from the environment."""
    script_dir = Path(__file__).resolve().parent
    v1 = script_dir.parent
    hawk = v1.parent.parent.parent
    link_dir = Path(os.environ.get("GCR16_LINK_DIR", str(hawk / "isaac_sim_test" / "3d")))
    out_dir = Path(os.environ.get("GCR16_OUT", str(v1 / "meshes")))
    os.makedirs(out_dir, exist_ok=True)
    return link_dir, out_dir


def _write_stl(shape, out_path, deflection=1.5):
    """Tessellate a shape and write STL. deflection is in STEP units (mm)."""
    try:
        shape.exportStl(str(out_path), deflection)
    except TypeError:
        shape.exportStl(str(out_path))
    bb = shape.BoundBox
    print(
        "Wrote",
        out_path,
        "V",
        round(getattr(shape, "Volume", 0.0), 1),
        "bbox",
        round(bb.XMin, 1),
        round(bb.XMax, 1),
        round(bb.YMin, 1),
        round(bb.YMax, 1),
        round(bb.ZMin, 1),
        round(bb.ZMax, 1),
    )


def _centroid(shape):
    """Return the bbox center of a FreeCAD shape."""
    bb = shape.BoundBox
    return (
        0.5 * (bb.XMin + bb.XMax),
        0.5 * (bb.YMin + bb.YMax),
        0.5 * (bb.ZMin + bb.ZMax),
    )


def _dist(a, b):
    """Return Euclidean distance between two 3-vectors."""
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def _fc_matrix(origin, x_axis, y_axis, z_axis):
    """Build a FreeCAD 4x4 from origin + orthonormal axes."""
    import FreeCAD

    m = FreeCAD.Matrix()
    m.A11, m.A21, m.A31 = x_axis
    m.A12, m.A22, m.A32 = y_axis
    m.A13, m.A23, m.A33 = z_axis
    m.A14, m.A24, m.A34 = origin
    return m


def _fusion_to_ros_matrix():
    """Map Fusion Y-up (x, y, z) to ROS Z-up (x, -z, y)."""
    import FreeCAD

    m = FreeCAD.Matrix()
    m.A11, m.A12, m.A13 = 1.0, 0.0, 0.0
    m.A21, m.A22, m.A23 = 0.0, 0.0, -1.0
    m.A31, m.A32, m.A33 = 0.0, 1.0, 0.0
    return m


def _translate_matrix(offset):
    """Return a translation matrix for an (x, y, z) millimetre offset."""
    import FreeCAD

    m = FreeCAD.Matrix()
    m.A14, m.A24, m.A34 = offset
    return m


def _ros_pin_mm(index):
    """Return the measured ROS pin for link index 0..6, in millimetres."""
    script_dir = Path(__file__).resolve().parent
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))
    from gcr16_pivots import PIVOTS

    pin = PIVOTS[index]["pin"]
    return (pin[0] * 1000.0, pin[1] * 1000.0, pin[2] * 1000.0)


def _instance_matrix(step_path):
    """Return T_asm * T_local^-1 from the first ITEM_DEFINED_TRANSFORMATION."""
    script_dir = Path(__file__).resolve().parent
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))
    from read_joint_indicators import (  # noqa: WPS433
        StepModel,
        _parse_entities,
        _parse_ref,
    )

    model = StepModel(_parse_entities(step_path.read_text(encoding="utf-8", errors="replace")))
    for _eid, ent in model.refs_of_type("ITEM_DEFINED_TRANSFORMATION"):
        refs = [_parse_ref(a) for a in ent["args"] if _parse_ref(a) is not None]
        if len(refs) < 2:
            continue
        local_pl, asm_pl = refs[-2], refs[-1]
        o_l, x_l, y_l, z_l = model.placement(local_pl)
        o_a, x_a, y_a, z_a = model.placement(asm_pl)
        t_local = _fc_matrix(o_l, x_l, y_l, z_l)
        t_asm = _fc_matrix(o_a, x_a, y_a, z_a)
        return t_asm.multiply(t_local.inverse()), o_a
    return None, (0.0, 0.0, 0.0)


def _load_shape(step_path):
    """Load a STEP with Part.read (no extra CLI files)."""
    import Part

    shape = Part.Shape()
    shape.read(str(step_path))
    return shape


def _to_fusion_world(shape, step_path):
    """Return the solid in Fusion assembly millimetres."""
    t_inst, asm_origin = _instance_matrix(step_path)
    raw = shape
    if t_inst is None:
        print("  no assembly placement — using Part.read as-is")
        return raw, asm_origin
    placed = raw.transformGeometry(t_inst)
    c_raw = _centroid(raw)
    c_pl = _centroid(placed)
    print(
        "  Part.read centroid",
        tuple(round(c, 2) for c in c_raw),
        "placed",
        tuple(round(c, 2) for c in c_pl),
        "asm",
        tuple(round(c, 2) for c in asm_origin),
    )
    if _dist(c_pl, asm_origin) <= _dist(c_raw, asm_origin):
        print("  using assembly placement")
        return placed, asm_origin
    print("  Part.read already closer to instance — keeping raw")
    return raw, asm_origin


def _to_link_local(shape, index):
    """Map Fusion-world shape to ROS link-local millimetres."""
    world = shape.transformGeometry(_fusion_to_ros_matrix())
    pin = _ros_pin_mm(index)
    local = world.transformGeometry(
        _translate_matrix((-pin[0], -pin[1], -pin[2]))
    )
    print(
        "  ROS world centroid",
        tuple(round(c, 2) for c in _centroid(world)),
        "pin mm",
        tuple(round(c, 2) for c in pin),
        "local centroid",
        tuple(round(c, 2) for c in _centroid(local)),
    )
    return world, local


def export_one(step_path, out_stl, index):
    """Convert one GCR16-Jn.step into a link-local ROS STL. Return world shape."""
    print("STEP", step_path)
    shape = _load_shape(step_path)
    if shape.isNull() or not list(shape.Solids):
        print("WARNING: no solids in", step_path)
        return None
    fusion, _asm = _to_fusion_world(shape, step_path)
    world, local = _to_link_local(fusion, index)
    _write_stl(local, out_stl)
    return world


def main():
    """Export J0-J6 link-local STLs plus a combined ROS-world GCR16-all.stl."""
    import Part

    link_dir, out_dir = _env_paths()
    print("LINK_DIR", link_dir)
    print("OUT     ", out_dir)
    worlds = []
    for index in range(7):
        step_path = link_dir / f"GCR16-J{index}.step"
        if not step_path.is_file():
            alt = link_dir / f"GCR16-J{index}.stp"
            step_path = alt if alt.is_file() else step_path
        if not step_path.is_file():
            print("MISSING", step_path)
            continue
        world = export_one(step_path, out_dir / f"GCR16-J{index}.stl", index)
        if world is not None:
            worlds.append(world)
    if worlds:
        _write_stl(Part.Compound(worlds), out_dir / "GCR16-all.stl")
    else:
        print("No per-link STEPs converted.")
        sys.exit(2)
    print("Done", len(worlds), "links (per-link STLs are LINK-LOCAL, all.stl is ROS world)")


if __name__ == "__main__":
    main()
