#!/usr/bin/env python3
"""Export GCR16-J0..J6 solids from the STEP assembly to STL (run via FreeCAD cmd).

Meshes stay in the STEP assembly frame so the assembled URDF can stack them at origin.
"""

import os
import sys


def _export_label_to_stl(doc, out_dir, label_prefix, out_name):
    """Find a document object whose label starts with label_prefix and write STL."""
    matches = []
    for obj in doc.Objects:
        label = getattr(obj, "Label", "") or ""
        name = getattr(obj, "Name", "") or ""
        if label.startswith(label_prefix) or name.startswith(label_prefix):
            if hasattr(obj, "Shape") and not obj.Shape.isNull():
                matches.append(obj)
    if not matches:
        print("WARNING: no shape for", label_prefix)
        return False
    # Prefer the largest solid if the part imported as several bits.
    obj = max(matches, key=lambda o: o.Shape.Volume if hasattr(o.Shape, "Volume") else 0)
    out_path = os.path.join(out_dir, out_name)
    obj.Shape.exportStl(out_path)
    print("Wrote", out_path, "from", obj.Label, "volume", getattr(obj.Shape, "Volume", "?"))
    return True


def main():
    """Import STEP in FreeCAD and write one STL per GCR16-Jn part."""
    if len(sys.argv) < 3:
        print("Usage: freecadcmd export_meshes_freecad.py STEP_PATH OUT_DIR")
        sys.exit(1)

    step_path = sys.argv[1]
    out_dir = sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    import FreeCAD  # noqa: F401
    import Import

    doc = FreeCAD.newDocument("gcr16_export")
    Import.insert(step_path, doc.Name)
    FreeCAD.setActiveDocument(doc.Name)

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
        if _export_label_to_stl(doc, out_dir, prefix, stl_name):
            ok += 1
    print("Exported", ok, "of", len(expected), "meshes to", out_dir)
    if ok == 0:
        print("Objects in document:")
        for obj in doc.Objects:
            print(" ", obj.Name, obj.Label, type(obj).__name__)
        sys.exit(2)


if __name__ == "__main__":
    main()
