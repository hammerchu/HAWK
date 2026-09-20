#!/usr/bin/env python3
"""Tessellate windows.step into ROS-metre Z-up STL (same law as the DXF path).

Run on the sim box with FreeCAD. Do not pass the STEP as a FreeCADCmd argument.

  WINDOW_STEP=.../windows.step WINDOW_STL_OUT=.../windows_ros.stl \\
    freecadcmd isaac_sim_test/window_path/export_window_stl_freecad.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _paths():
    """Resolve STEP input and STL output from env, with HAWK defaults."""
    here = Path(__file__).resolve().parent
    hawk_3d = here.parent / "3d"
    generated = here / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    step = Path(os.environ.get("WINDOW_STEP", str(hawk_3d / "windows.step")))
    out = Path(os.environ.get("WINDOW_STL_OUT", str(generated / "windows_ros.stl")))
    return step, out


def _fusion_to_ros_matrix():
    """Map Fusion Y-up (x, y, z) to ROS Z-up (x, -z, y)."""
    import FreeCAD

    m = FreeCAD.Matrix()
    m.A11, m.A12, m.A13 = 1.0, 0.0, 0.0
    m.A21, m.A22, m.A23 = 0.0, 0.0, -1.0
    m.A31, m.A32, m.A33 = 0.0, 1.0, 0.0
    return m


def _scale_matrix(scale):
    """Return a uniform scale matrix (CAD cm -> metres is 0.01)."""
    import FreeCAD

    m = FreeCAD.Matrix()
    m.A11 = m.A22 = m.A33 = scale
    return m


def _write_stl(shape, out_path, deflection=0.15):
    """Tessellate and write STL. deflection is in current shape units (metres)."""
    try:
        shape.exportStl(str(out_path), deflection)
    except TypeError:
        shape.exportStl(str(out_path))
    bb = shape.BoundBox
    print(
        "Wrote",
        out_path,
        "bbox",
        round(bb.XMin, 3),
        round(bb.XMax, 3),
        round(bb.YMin, 3),
        round(bb.YMax, 3),
        round(bb.ZMin, 3),
        round(bb.ZMax, 3),
    )


def main():
    """Load the Fusion window STEP, scale cm->m, Y-up->Z-up, write STL."""
    import Part

    step_path, out_path = _paths()
    if not step_path.is_file():
        print("MISSING", step_path)
        sys.exit(2)
    print("STEP", step_path)
    shape = Part.Shape()
    shape.read(str(step_path))
    if shape.isNull():
        print("Empty STEP")
        sys.exit(2)
    # Raw STEP numbers are centimetres. Same 0.01 + Rx90 as the DXF path.
    metres = shape.transformGeometry(_scale_matrix(0.01))
    ros = metres.transformGeometry(_fusion_to_ros_matrix())
    _write_stl(ros, out_path)
    print(
        "CAD-local ROS m. In Isaac parent this STL under /World/window_frame "
        "at identity — do NOT add another cm/Y-up convert."
    )


if __name__ == "__main__":
    main()
