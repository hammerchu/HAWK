#!/usr/bin/env python3
"""Rewrite FreeCAD STLs as binary STL so RViz/Assimp can load them."""

import os
import struct
import sys
from pathlib import Path


def _read_ascii_stl(text):
    """Parse an ASCII STL into a list of 3-vertex triangles."""
    tris = []
    verts = []
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("vertex"):
            parts = line.split()
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(verts) == 3:
                tris.append(tuple(verts))
                verts = []
    return tris


def _read_binary_stl(data):
    """Parse a binary STL into triangles. Returns None if the file is not binary."""
    if len(data) < 84:
        return None
    if data.lstrip().lower().startswith(b"solid") and b"facet" in data[:512].lower():
        return None
    count = struct.unpack_from("<I", data, 80)[0]
    need = 84 + count * 50
    if count <= 0 or need > len(data) + 50:
        return None
    tris = []
    offset = 84
    for _ in range(count):
        if offset + 50 > len(data):
            break
        nx, ny, nz, x1, y1, z1, x2, y2, z2, x3, y3, z3 = struct.unpack_from(
            "<12f", data, offset
        )
        tris.append(((x1, y1, z1), (x2, y2, z2), (x3, y3, z3)))
        offset += 50
    return tris


def _cross(a, b):
    """Return the cross product of two 3D vectors."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normal(tri):
    """Unit face normal for one triangle."""
    a, b, c = tri
    ux = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    vx = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    n = _cross(ux, vx)
    length = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (n[0] / length, n[1] / length, n[2] / length)


def write_binary_stl(path, triangles):
    """Write a spec-compliant binary STL."""
    header = b"HAWK GCR16 binary STL".ljust(80, b"\0")
    payload = [header, struct.pack("<I", len(triangles))]
    for tri in triangles:
        n = _normal(tri)
        payload.append(struct.pack("<3f", *n))
        for vert in tri:
            payload.append(struct.pack("<3f", *vert))
        payload.append(struct.pack("<H", 0))
    path.write_bytes(b"".join(payload))


def convert_stl(path):
    """Load one STL and overwrite it as binary. Return triangle count."""
    data = path.read_bytes()
    tris = _read_binary_stl(data)
    if tris is None:
        try:
            tris = _read_ascii_stl(data.decode("utf-8", errors="ignore"))
        except Exception:
            tris = []
    if not tris:
        print("SKIP empty/unreadable", path)
        return 0
    write_binary_stl(path, tris)
    print("OK", path.name, "triangles", len(tris), "bytes", path.stat().st_size)
    return len(tris)


def main():
    """Convert every GCR16-*.stl in the given directory (default: ../meshes)."""
    mesh_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "meshes"
    files = sorted(mesh_dir.glob("GCR16-*.stl"))
    if not files:
        print("No GCR16-*.stl in", mesh_dir)
        sys.exit(1)
    total = 0
    for path in files:
        total += convert_stl(path)
    if total == 0:
        sys.exit(2)


if __name__ == "__main__":
    main()
