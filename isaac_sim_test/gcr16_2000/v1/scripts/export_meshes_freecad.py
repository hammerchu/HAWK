#!/usr/bin/env python3
"""Export GCR16 meshes from STEP using FreeCAD Part.read (no CLI extra files).

FreeCADCmd treats extra arguments as documents to open, which crashed 0.19 on
this SolidWorks STEP. Paths come from GCR16_STEP and GCR16_OUT.
"""

import os
import sys


def _env_paths():
    """Read STEP and output dir from the environment."""
    step_path = os.environ.get("GCR16_STEP", "")
    out_dir = os.environ.get("GCR16_OUT", "")
    if not step_path or not out_dir:
        print("Set GCR16_STEP and GCR16_OUT. Do not pass them as FreeCADCmd args.")
        sys.exit(1)
    if not os.path.isfile(step_path):
        print("STEP not found:", step_path)
        sys.exit(1)
    os.makedirs(out_dir, exist_ok=True)
    return step_path, out_dir


def _write_stl(shape, out_path, deflection=0.4):
    """Tessellate a shape and write STL. deflection is in STEP units (mm)."""
    try:
        shape.exportStl(out_path, deflection)
    except TypeError:
        shape.exportStl(out_path)
    print("Wrote", out_path, "volume", getattr(shape, "Volume", "?"))


def _solids_from_part_read(step_path):
    """Load STEP through OCCT Part.read — more stable than ImportOCAF on 0.19."""
    import Part

    shape = Part.Shape()
    shape.read(step_path)
    solids = list(shape.Solids)
    print("Part.read solids:", len(solids), "volume", shape.Volume)
    return shape, solids


def _export_named_from_doc(doc, out_dir):
    """If OCAF import survived, export objects named GCR16-Jn."""
    import FreeCAD

    expected = [
        ("GCR16-J0", "GCR16-J0.stl"),
        ("GCR16-J1", "GCR16-J1.stl"),
        ("GCR16-J2", "GCR16-J2.stl"),
        ("GCR16-J3", "GCR16-J3.stl"),
        ("GCR16-J4", "GCR16-J4.stl"),
        ("GCR16-J5", "GCR16-J5.stl"),
        ("GCR16-J6", "GCR16-J6.stl"),
    ]
    ok = 0
    for prefix, stl_name in expected:
        matches = []
        for obj in doc.Objects:
            label = getattr(obj, "Label", "") or ""
            name = getattr(obj, "Name", "") or ""
            if (label.startswith(prefix) or name.startswith(prefix)) and hasattr(obj, "Shape"):
                if not obj.Shape.isNull():
                    matches.append(obj)
        if not matches:
            print("WARNING: no shape for", prefix)
            continue
        obj = max(matches, key=lambda o: o.Shape.Volume if hasattr(o.Shape, "Volume") else 0)
        _write_stl(obj.Shape, os.path.join(out_dir, stl_name))
        ok += 1
    return ok


def _export_solids_sorted(solids, out_dir):
    """Name the 7 biggest solids J0..J6 by rising bbox Z (base first)."""
    usable = [s for s in solids if s.Volume > 1.0]
    usable.sort(key=lambda s: s.Volume, reverse=True)
    usable = usable[:7]
    usable.sort(key=lambda s: s.BoundBox.ZMin)
    names = [
        "GCR16-J0.stl",
        "GCR16-J1.stl",
        "GCR16-J2.stl",
        "GCR16-J3.stl",
        "GCR16-J4.stl",
        "GCR16-J5.stl",
        "GCR16-J6.stl",
    ]
    if len(usable) < 7:
        print("Only", len(usable), "solids — writing what we have.")
    for shape, name in zip(usable, names):
        bb = shape.BoundBox
        print(
            name,
            "ZMin",
            round(bb.ZMin, 2),
            "ZMax",
            round(bb.ZMax, 2),
            "V",
            round(shape.Volume, 1),
        )
        _write_stl(shape, os.path.join(out_dir, name))
    return len(usable)


def main():
    """Export per-link STLs, then a combined STL, without feeding paths to FreeCADCmd."""
    step_path, out_dir = _env_paths()
    print("STEP", step_path)
    print("OUT ", out_dir)

    import FreeCAD
    import Part

    whole, solids = _solids_from_part_read(step_path)
    _write_stl(whole, os.path.join(out_dir, "GCR16-all.stl"))

    ok = 0
    if solids:
        ok = _export_solids_sorted(solids, out_dir)
    else:
        print("No solids from Part.read; not using Import.insert (segfaults on 0.19).")

    print("Per-link meshes:", ok)
    if ok == 0 and not os.path.isfile(os.path.join(out_dir, "GCR16-all.stl")):
        sys.exit(2)


if __name__ == "__main__":
    main()
