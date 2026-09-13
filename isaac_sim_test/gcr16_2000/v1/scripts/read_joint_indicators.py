#!/usr/bin/env python3
"""Read named jointN_root indicator cylinders from a Fusion-exported STEP.

Reusable: add joint3_root ... joint6_root the same way and re-run.
Each indicator must be a cylinder. Center = pin, axis = spin direction.

  python3 scripts/read_joint_indicators.py ../../3d/GCR16-2000_joint_root.step
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path


JOINT_NAME_RE = re.compile(r"^joint(\d+)_root$", re.IGNORECASE)
BASE_NAME_RE = re.compile(r"^base_root$", re.IGNORECASE)


def _indicator_identity(product_name):
    """Return (sort_index, part_name, urdf_name) for a named indicator, or None."""
    if BASE_NAME_RE.match(product_name):
        return 0, "base_root", "base_link"
    match = JOINT_NAME_RE.match(product_name)
    if match:
        index = int(match.group(1))
        return index, f"joint{index}_root", f"gcr16_joint{index}"
    return None


def _tokenize_step_args(blob):
    """Split a STEP argument list into top-level tokens."""
    tokens = []
    buf = []
    depth = 0
    in_str = False
    i = 0
    while i < len(blob):
        ch = blob[i]
        if in_str:
            buf.append(ch)
            if ch == "'":
                in_str = False
            i += 1
            continue
        if ch == "'":
            in_str = True
            buf.append(ch)
            i += 1
            continue
        if ch == "(":
            depth += 1
            buf.append(ch)
        elif ch == ")":
            depth -= 1
            buf.append(ch)
        elif ch == "," and depth == 0:
            tokens.append("".join(buf).strip())
            buf = []
        else:
            buf.append(ch)
        i += 1
    tail = "".join(buf).strip()
    if tail:
        tokens.append(tail)
    return tokens


def _parse_ref(token):
    """Return entity id int if token is #N, else None."""
    token = token.strip()
    if token.startswith("#"):
        return int(token[1:])
    return None


def _parse_float_tuple(token):
    """Parse (a,b,c) or a single number into a float tuple."""
    token = token.strip()
    if token.startswith("(") and token.endswith(")"):
        inner = token[1:-1]
        parts = _tokenize_step_args(inner)
        return tuple(float(p) for p in parts)
    return (float(token),)


def _parse_entities(text):
    """Parse #id=TYPE(args); and complex #id=(...); records from DATA."""
    start = text.find("DATA;")
    end = text.rfind("ENDSEC;")
    if start < 0 or end < 0:
        raise ValueError("STEP file has no DATA section")
    body = text[start + 5 : end]
    entities = {}
    for match in re.finditer(
        r"#(\d+)\s*=\s*(.+?);", body, flags=re.DOTALL
    ):
        eid = int(match.group(1))
        raw = " ".join(match.group(2).split())
        if raw.startswith("("):
            entities[eid] = {"type": "COMPLEX", "raw": raw, "args": []}
            continue
        type_end = raw.find("(")
        if type_end < 0:
            entities[eid] = {"type": raw.strip(), "raw": raw, "args": []}
            continue
        etype = raw[:type_end].strip()
        args_blob = raw[type_end + 1 : raw.rfind(")")]
        entities[eid] = {
            "type": etype,
            "raw": raw,
            "args": _tokenize_step_args(args_blob),
        }
    return entities


def _unquote(token):
    """Strip STEP single quotes from a string token."""
    token = token.strip()
    if token.startswith("'") and token.endswith("'") and len(token) >= 2:
        return token[1:-1]
    return token


def _vec_sub(a, b):
    """Return a - b."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_add(a, b):
    """Return a + b."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_scale(a, s):
    """Return a * s."""
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    """Return the dot product."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    """Return the cross product."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a):
    """Return Euclidean length."""
    return math.sqrt(_dot(a, a))


def _unit(a, fallback=(0.0, 0.0, 1.0)):
    """Return a unit vector, or fallback if nearly zero."""
    n = _norm(a)
    if n < 1e-12:
        return fallback
    return _vec_scale(a, 1.0 / n)


def _snap_axis(axis, tol=1e-3):
    """Snap a near-world-axis vector to ±X/±Y/±Z."""
    u = _unit(axis)
    best = u
    best_dot = -1.0
    for cand in (
        (1, 0, 0),
        (-1, 0, 0),
        (0, 1, 0),
        (0, -1, 0),
        (0, 0, 1),
        (0, 0, -1),
    ):
        d = abs(_dot(u, cand))
        if d > best_dot:
            best_dot = d
            best = cand if _dot(u, cand) >= 0 else _vec_scale(cand, -1.0)
    if best_dot >= 1.0 - tol:
        return best
    return tuple(round(c, 9) for c in u)


class StepModel:
    """In-memory STEP AP214 graph for joint-indicator cylinders."""

    def __init__(self, entities):
        """Index parsed STEP entities."""
        self.entities = entities

    def get(self, eid):
        """Return an entity dict or raise."""
        if eid not in self.entities:
            raise KeyError(f"missing #{eid}")
        return self.entities[eid]

    def cartesian(self, eid):
        """Return a CARTESIAN_POINT as (x, y, z)."""
        ent = self.get(eid)
        coords = _parse_float_tuple(ent["args"][-1])
        if len(coords) != 3:
            raise ValueError(f"#{eid} is not a 3D point")
        return coords

    def direction(self, eid):
        """Return a DIRECTION as a unit (x, y, z)."""
        ent = self.get(eid)
        coords = _parse_float_tuple(ent["args"][-1])
        if len(coords) != 3:
            raise ValueError(f"#{eid} is not a 3D direction")
        return _unit(coords)

    def placement(self, eid):
        """Return origin, z-axis, x-axis for an AXIS2_PLACEMENT_3D."""
        ent = self.get(eid)
        if ent["type"] != "AXIS2_PLACEMENT_3D":
            raise ValueError(f"#{eid} is {ent['type']}, not AXIS2_PLACEMENT_3D")
        origin = self.cartesian(_parse_ref(ent["args"][1]))
        z_axis = self.direction(_parse_ref(ent["args"][2]))
        x_raw = self.direction(_parse_ref(ent["args"][3]))
        x_axis = _unit(_vec_sub(x_raw, _vec_scale(z_axis, _dot(x_raw, z_axis))))
        y_axis = _unit(_cross(z_axis, x_axis))
        x_axis = _unit(_cross(y_axis, z_axis))
        return origin, x_axis, y_axis, z_axis

    def apply_placement(self, eid, point, direction=None):
        """Map a local point (and optional direction) through a placement."""
        origin, x_axis, y_axis, z_axis = self.placement(eid)
        world = _vec_add(
            origin,
            _vec_add(
                _vec_add(
                    _vec_scale(x_axis, point[0]),
                    _vec_scale(y_axis, point[1]),
                ),
                _vec_scale(z_axis, point[2]),
            ),
        )
        if direction is None:
            return world, None
        dworld = _vec_add(
            _vec_add(
                _vec_scale(x_axis, direction[0]),
                _vec_scale(y_axis, direction[1]),
            ),
            _vec_scale(z_axis, direction[2]),
        )
        return world, _unit(dworld)

    def compose_instance(self, local_pl, asm_pl, point, direction):
        """Map part-space point/dir into assembly using ITEM_DEFINED_TRANSFORMATION."""
        # Inverse of local placement, then assembly placement.
        origin_l, x_l, y_l, z_l = self.placement(local_pl)
        local = _vec_sub(point, origin_l)
        part = (
            _dot(local, x_l),
            _dot(local, y_l),
            _dot(local, z_l),
        )
        dir_part = (
            _dot(direction, x_l),
            _dot(direction, y_l),
            _dot(direction, z_l),
        )
        return self.apply_placement(asm_pl, part, dir_part)

    def refs_of_type(self, etype):
        """Yield (id, entity) for a STEP type name."""
        for eid, ent in self.entities.items():
            if ent["type"] == etype:
                yield eid, ent

    def product_name(self, eid):
        """Best-effort PRODUCT / PRODUCT_DEFINITION name."""
        ent = self.get(eid)
        if not ent["args"]:
            return ""
        return _unquote(ent["args"][0])


def _collect_face_ids(model, shell_eid, out):
    """Recursively collect ADVANCED_FACE ids under a shell or brep."""
    ent = model.get(shell_eid)
    etype = ent["type"]
    if etype == "ADVANCED_FACE":
        out.append(shell_eid)
        return
    for arg in ent["args"]:
        if arg.startswith("(") and arg.endswith(")"):
            for tok in _tokenize_step_args(arg[1:-1]):
                ref = _parse_ref(tok)
                if ref is not None:
                    _collect_face_ids(model, ref, out)
        else:
            ref = _parse_ref(arg)
            if ref is not None and model.get(ref)["type"] not in (
                "CARTESIAN_POINT",
                "DIRECTION",
                "AXIS2_PLACEMENT_3D",
            ):
                _collect_face_ids(model, ref, out)


def _cylinder_on_faces(model, face_ids):
    """Pick the indicator cylinder from a solid's faces (center + axis + length)."""
    cylinders = []
    planes = []
    for fid in face_ids:
        face = model.get(fid)
        # ADVANCED_FACE('', bounds, surface, orient)
        surf_id = None
        for arg in reversed(face["args"]):
            ref = _parse_ref(arg)
            if ref is None:
                continue
            stype = model.get(ref)["type"]
            if stype in ("CYLINDRICAL_SURFACE", "PLANE"):
                surf_id = ref
                break
        if surf_id is None:
            continue
        surf = model.get(surf_id)
        pl_id = _parse_ref(surf["args"][1])
        origin, _x, _y, z_axis = model.placement(pl_id)
        if surf["type"] == "CYLINDRICAL_SURFACE":
            radius = float(surf["args"][2])
            cylinders.append(
                {"origin": origin, "axis": z_axis, "radius": radius}
            )
        elif surf["type"] == "PLANE":
            planes.append({"origin": origin, "normal": z_axis})
    if not cylinders:
        return None
    # Indicator parts are a single cylinder; if several, take the longest cap span.
    best = None
    best_len = -1.0
    for cyl in cylinders:
        caps = []
        for pln in planes:
            if abs(_dot(_unit(pln["normal"]), _unit(cyl["axis"]))) < 0.98:
                continue
            rel = _vec_sub(pln["origin"], cyl["origin"])
            caps.append(_dot(rel, _unit(cyl["axis"])))
        if len(caps) >= 2:
            length = max(caps) - min(caps)
            pin = _vec_add(
                cyl["origin"],
                _vec_scale(_unit(cyl["axis"]), 0.5 * (max(caps) + min(caps))),
            )
        else:
            length = 0.0
            pin = cyl["origin"]
        if length > best_len:
            best_len = length
            best = {
                "pin": pin,
                "axis": _unit(cyl["axis"]),
                "radius": cyl["radius"],
                "length": abs(length),
            }
    return best


def _brep_root_for_product_def(model, prod_def_id):
    """Find this part's ABR only — do not walk assembly / sibling transforms."""
    pds_ids = []
    for sid, pds in model.refs_of_type("PRODUCT_DEFINITION_SHAPE"):
        refs = [_parse_ref(a) for a in pds["args"] if _parse_ref(a) is not None]
        # Skip NAUO-linked PDS (those are instance, not the part definition).
        if prod_def_id in refs and not any(
            model.get(r)["type"] == "NEXT_ASSEMBLY_USAGE_OCCURRENCE" for r in refs
        ):
            pds_ids.append(sid)
    part_srs = []
    for _sdr_id, sdr in model.refs_of_type("SHAPE_DEFINITION_REPRESENTATION"):
        a = _parse_ref(sdr["args"][0])
        b = _parse_ref(sdr["args"][1])
        if a in pds_ids and b is not None:
            part_srs.append(b)
        elif b in pds_ids and a is not None:
            part_srs.append(a)
    abr = []
    for srr_id, srr in model.refs_of_type("SHAPE_REPRESENTATION_RELATIONSHIP"):
        if srr["type"] != "SHAPE_REPRESENTATION_RELATIONSHIP":
            continue
        refs = [_parse_ref(a) for a in srr["args"] if _parse_ref(a) is not None]
        if not any(r in part_srs for r in refs):
            continue
        for ref in refs:
            if model.get(ref)["type"] == "ADVANCED_BREP_SHAPE_REPRESENTATION":
                abr.append(ref)
    return abr


def _instance_transform_for_product_def(model, prod_def_id):
    """Return (local_placement_id, assembly_placement_id) for a part instance."""
    nauo_ids = []
    for nid, nauo in model.refs_of_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE"):
        refs = [_parse_ref(a) for a in nauo["args"] if _parse_ref(a) is not None]
        if prod_def_id in refs:
            nauo_ids.append(nid)
    cdsr_ids = []
    for _pid, pds in model.refs_of_type("PRODUCT_DEFINITION_SHAPE"):
        refs = [_parse_ref(a) for a in pds["args"] if _parse_ref(a) is not None]
        if any(n in refs for n in nauo_ids):
            for cid, cdsr in model.refs_of_type(
                "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"
            ):
                crefs = [
                    _parse_ref(a) for a in cdsr["args"] if _parse_ref(a) is not None
                ]
                if _pid in crefs:
                    cdsr_ids.append(cid)
    for cid in cdsr_ids:
        cdsr = model.get(cid)
        rel = _parse_ref(cdsr["args"][0])
        if rel is None:
            continue
        raw = model.get(rel)["raw"] if rel in model.entities else ""
        idt = re.search(r"ITEM_DEFINED_TRANSFORMATION\([^)]*?(#\d+)\s*,\s*(#\d+)\)", raw)
        if idt:
            return int(idt.group(1)[1:]), int(idt.group(2)[1:])
        # Non-complex ITEM_DEFINED_TRANSFORMATION referenced from the relationship.
        for ref in re.findall(r"#(\d+)", raw):
            rid = int(ref)
            if model.get(rid)["type"] == "ITEM_DEFINED_TRANSFORMATION":
                args = model.get(rid)["args"]
                local_pl = _parse_ref(args[-2])
                asm_pl = _parse_ref(args[-1])
                return local_pl, asm_pl
    return None, None


def _fusion_y_up_to_ros(vec):
    """Map Fusion Y-up (x, y, z) to ROS/URDF Z-up (x, -z, y)."""
    return (vec[0], -vec[2], vec[1])


def read_joint_indicators(step_path):
    """Return a sorted list of joint indicator dicts from a STEP path."""
    text = Path(step_path).read_text(encoding="utf-8", errors="replace")
    model = StepModel(_parse_entities(text))
    joints = []
    for eid, ent in model.refs_of_type("PRODUCT"):
        name = model.product_name(eid)
        ident = _indicator_identity(name)
        if ident is None:
            continue
        index, part_name, urdf_name = ident
        # PRODUCT -> PRODUCT_DEFINITION via formation
        prod_defs = []
        for _fid, form in model.refs_of_type("PRODUCT_DEFINITION_FORMATION"):
            refs = [_parse_ref(a) for a in form["args"] if _parse_ref(a) is not None]
            if eid in refs:
                for pdid, pdef in model.refs_of_type("PRODUCT_DEFINITION"):
                    prefs = [
                        _parse_ref(a) for a in pdef["args"] if _parse_ref(a) is not None
                    ]
                    if _fid in prefs:
                        prod_defs.append(pdid)
        if not prod_defs:
            continue
        prod_def_id = prod_defs[0]
        abr_ids = _brep_root_for_product_def(model, prod_def_id)
        local_cyl = None
        for abr_id in abr_ids:
            faces = []
            _collect_face_ids(model, abr_id, faces)
            local_cyl = _cylinder_on_faces(model, faces)
            if local_cyl:
                break
        if not local_cyl:
            raise RuntimeError(f"{name}: no cylinder found on the indicator solid")
        local_pl, asm_pl = _instance_transform_for_product_def(model, prod_def_id)
        if local_pl is None or asm_pl is None:
            raise RuntimeError(f"{name}: no assembly placement")
        pin, axis = model.compose_instance(
            local_pl, asm_pl, local_cyl["pin"], local_cyl["axis"]
        )
        joints.append(
            {
                "name": part_name,
                "joint": urdf_name,
                "index": index,
                "pin_mm": pin,
                "axis": _snap_axis(axis),
                "axis_raw": axis,
                "radius_mm": local_cyl["radius"],
                "length_mm": local_cyl["length"],
            }
        )
    joints.sort(key=lambda j: j["index"])
    return joints


def _fmt_vec(vec, digits=4):
    """Format a 3-vector for the terminal."""
    return " ".join(f"{c:.{digits}f}" for c in vec)


def main(argv=None):
    """CLI: print pin/axis for every jointN_root cylinder in a STEP."""
    parser = argparse.ArgumentParser(
        description="Read jointN_root indicator cylinders from a STEP file."
    )
    here = Path(__file__).resolve()
    default_step = here.parents[2] / "3d" / "GCR16-2000_joint_root.step"
    parser.add_argument(
        "step",
        nargs="?",
        default=str(default_step),
        help="Fusion STEP that contains jointN_root cylinders",
    )
    parser.add_argument(
        "--ros-z-up",
        action="store_true",
        help="Also print Fusion Y-up mapped to ROS Z-up (x, y, z) -> (x, -z, y)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON instead of a table",
    )
    args = parser.parse_args(argv)
    step = Path(args.step)
    if not step.is_file():
        print("STEP not found:", step, file=sys.stderr)
        return 2
    joints = read_joint_indicators(step)
    if not joints:
        print("No jointN_root products in", step, file=sys.stderr)
        return 1
    rows = []
    for j in joints:
        row = {
            "name": j["name"],
            "joint": j["joint"],
            "pin_mm": [round(c, 4) for c in j["pin_mm"]],
            "axis": [round(c, 6) for c in j["axis"]],
            "radius_mm": round(j["radius_mm"], 4),
            "length_mm": round(j["length_mm"], 4),
        }
        if args.ros_z_up:
            row["pin_mm_ros"] = [round(c, 4) for c in _fusion_y_up_to_ros(j["pin_mm"])]
            row["axis_ros"] = [
                round(c, 6) for c in _snap_axis(_fusion_y_up_to_ros(j["axis_raw"]))
            ]
        rows.append(row)
    if args.json:
        print(json.dumps(rows, indent=2))
        return 0
    print(f"# {step.name}: {len(rows)} indicator(s)")
    header = (
        f"{'name':<16} {'pin_mm (Fusion)':<32} {'axis':<16} {'r':>6} {'len':>8}"
    )
    print(header)
    for row in rows:
        print(
            f"{row['name']:<16} {_fmt_vec(row['pin_mm']):<32} "
            f"{_fmt_vec(row['axis'], 3):<16} {row['radius_mm']:6.2f} {row['length_mm']:8.2f}"
        )
        if args.ros_z_up:
            print(
                f"{'  ros Z-up':<16} {_fmt_vec(row['pin_mm_ros']):<32} "
                f"{_fmt_vec(row['axis_ros'], 3)}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
