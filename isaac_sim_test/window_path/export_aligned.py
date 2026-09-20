#!/usr/bin/env python3
"""Bake S (and L) path + a glass box into the same ROS metres frame as window_geo."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
REPO_TEST = PACKAGE_DIR.parent
if str(REPO_TEST) not in sys.path:
    sys.path.insert(0, str(REPO_TEST))

from window_path.cad_frame import (  # noqa: E402
    apply_window_pose,
    cad_cm_to_ros_m,
    fbx_child_orient_wxyz,
    load_window_pose,
)
from window_path.dxf_path import (  # noqa: E402
    densify_closed,
    load_l_rectangle_cm,
    load_s_rectangle_cm,
)


def generated_dir():
    """Return window_path/generated and create it if needed."""
    out = PACKAGE_DIR / "generated"
    out.mkdir(parents=True, exist_ok=True)
    return out


def corners_to_ros(corners_cm):
    """Convert a CAD-cm rectangle to CAD-local ROS metres."""
    return [cad_cm_to_ros_m(*p) for p in corners_cm]


def write_json(path, payload):
    """Write pretty JSON for Isaac / Python consumers."""
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_ascii_box_stl(path, corners_cad, thickness=0.04):
    """Write a thin box whose front face is the path rectangle (CAD-local ROS m)."""
    if len(corners_cad) != 4:
        raise ValueError("Glass box needs 4 CAD-local corners")
    front = list(corners_cad)
    back = [(x, y - thickness, z) for x, y, z in front]
    # 8 vertices: front BL BR TR TL, back BL BR TR TL
    verts = front + back

    def tri(a, b, c):
        """Return one ASCII STL facet from three vertex indices."""
        ax, ay, az = verts[a]
        bx, by, bz = verts[b]
        cx, cy, cz = verts[c]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        return (
            "  facet normal {nx:.7e} {ny:.7e} {nz:.7e}\n"
            "    outer loop\n"
            "      vertex {ax:.7e} {ay:.7e} {az:.7e}\n"
            "      vertex {bx:.7e} {by:.7e} {bz:.7e}\n"
            "      vertex {cx:.7e} {cy:.7e} {cz:.7e}\n"
            "    endloop\n"
            "  endfacet\n"
        ).format(
            nx=nx, ny=ny, nz=nz,
            ax=ax, ay=ay, az=az,
            bx=bx, by=by, bz=bz,
            cx=cx, cy=cy, cz=cz,
        )

    # Front (toward +Y / robot), back, and four sides.
    faces = [
        (0, 1, 2), (0, 2, 3),
        (5, 4, 7), (5, 7, 6),
        (1, 0, 4), (1, 4, 5),
        (2, 1, 5), (2, 5, 6),
        (3, 2, 6), (3, 6, 7),
        (0, 3, 7), (0, 7, 4),
    ]
    body = ["solid s_glass\n"]
    body.extend(tri(a, b, c) for a, b, c in faces)
    body.append("endsolid s_glass\n")
    path.write_text("".join(body), encoding="utf-8")


def _fmt_pt(pt):
    """Format a 3-tuple for USDA point3f."""
    return "({0:.6f}, {1:.6f}, {2:.6f})".format(*pt)


def write_window_usda(path, pose, s_cad, l_cad):
    """Write a Z-up metres USDA: parent Xform + S/L curves + FBX child xform."""
    tx, ty, tz = pose["translation_m"]
    w, x, y, z = fbx_child_orient_wxyz()
    scale = float(pose.get("fbx_scale", 0.01))
    fbx_rel = "../../3d/windows.fbx"
    s_pts = ",\n            ".join(_fmt_pt(p) for p in s_cad + [s_cad[0]])
    l_pts = ",\n            ".join(_fmt_pt(p) for p in l_cad + [l_cad[0]])
    text = f"""#usda 1.0
(
    defaultPrim = "window_frame"
    metersPerUnit = 1
    upAxis = "Z"
    doc = "Parent once. Path is already ROS m. FBX child still needs cm + Rx90."
)

def Xform "window_frame"
{{
    double3 xformOp:translate = ({tx:.6f}, {ty:.6f}, {tz:.6f})
    uniform token[] xformOpOrder = ["xformOp:translate"]

    def Xform "window_geo" (
        prepend references = @{fbx_rel}@
    )
    {{
        quatd xformOp:orient = ({w}, {x}, {y}, {z})
        double3 xformOp:scale = ({scale}, {scale}, {scale})
        uniform token[] xformOpOrder = ["xformOp:orient", "xformOp:scale"]
    }}

    def BasisCurves "S_path"
    {{
        uniform token type = "linear"
        int[] curveVertexCounts = [{len(s_cad) + 1}]
        point3f[] points = [
            {s_pts}
        ]
        float[] widths = [0.012]
        color3f[] primvars:displayColor = [(0.15, 0.95, 0.35)]
    }}

    def BasisCurves "L_path"
    {{
        uniform token type = "linear"
        int[] curveVertexCounts = [{len(l_cad) + 1}]
        point3f[] points = [
            {l_pts}
        ]
        float[] widths = [0.012]
        color3f[] primvars:displayColor = [(0.95, 0.75, 0.15)]
    }}
}}
"""
    path.write_text(text, encoding="utf-8")


def path_payload(name, corners_cm, pose, spacing_m):
    """Build the JSON block for one named path (CAD-local and world)."""
    cad = corners_to_ros(corners_cm)
    world = apply_window_pose(
        cad,
        pose["translation_m"],
        pose["rpy_rad"],
        pose["approach_offset_m"],
    )
    dense = densify_closed(world, spacing_m)
    return {
        "name": name,
        "corners_cad_m": [list(p) for p in cad],
        "corners_world_m": [list(p) for p in world],
        "waypoints_world_m": [list(p) for p in dense],
        "bbox_world_m": {
            "min": [min(p[i] for p in world) for i in range(3)],
            "max": [max(p[i] for p in world) for i in range(3)],
        },
    }


def export_all(pose_path=None):
    """Export JSON, glass STL, and USDA from the live DXF + window_frame.json."""
    pose = load_window_pose(pose_path)
    spacing = float(pose.get("spacing_m", 0.03))
    s_cm = load_s_rectangle_cm()
    l_cm = load_l_rectangle_cm()
    s_cad = corners_to_ros(s_cm)
    l_cad = corners_to_ros(l_cm)
    out = generated_dir()

    s_json = path_payload("S", s_cm, pose, spacing)
    l_json = path_payload("L", l_cm, pose, spacing)
    write_json(
        out / "S_path.json",
        {"pose": {k: pose[k] for k in ("parent_frame", "translation_m", "rpy_rad", "approach_offset_m")},
         "path": s_json},
    )
    write_json(
        out / "L_path.json",
        {"pose": {k: pose[k] for k in ("parent_frame", "translation_m", "rpy_rad", "approach_offset_m")},
         "path": l_json},
    )
    write_ascii_box_stl(out / "s_glass.stl", s_cad)
    write_window_usda(out / "window_frame.usda", pose, s_cad, l_cad)

    smin, smax = s_json["bbox_world_m"]["min"], s_json["bbox_world_m"]["max"]
    print("Wrote", out)
    print(
        "S world bbox x[{:.3f},{:.3f}] y[{:.3f},{:.3f}] z[{:.3f},{:.3f}] n={}".format(
            smin[0], smax[0], smin[1], smax[1], smin[2], smax[2],
            len(s_json["waypoints_world_m"]),
        )
    )
    print("Isaac: File -> Add ->", out / "window_frame.usda")
    print("If FBX comes in rotated twice, hide window_geo and Add s_glass.stl under window_frame.")
    return out


def main():
    """CLI: rebuild aligned path + window assets from DXF and window_frame.json."""
    parser = argparse.ArgumentParser(description="Export aligned window + S/L path.")
    parser.add_argument("--pose", default="", help="Override window_frame.json path")
    args = parser.parse_args()
    export_all(args.pose or None)


if __name__ == "__main__":
    main()
