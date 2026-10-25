"""Bake the supplied STEP field into a compact render mesh.

Uses OpenCASCADE through the `cadquery-ocp` Python package (LGPL) only at
asset-build time. The simulator itself reads the generated .rfield asset and
has no runtime CAD dependency.

Usage: python tools/bake_field.py [input.step] [output.rfield]
"""
from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


def hash_color(seed: int, base: tuple[float, float, float], jitter: float = 0.025):
    x = (seed * 1664525 + 1013904223) & 0xFFFFFFFF
    delta = ((x >> 8) & 0xFF) / 255.0 * jitter - jitter * 0.5
    return tuple(max(0.0, min(1.0, c + delta)) for c in base)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    input_path = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "26校内赛场地.STEP"
    output_path = Path(sys.argv[2]) if len(sys.argv) > 2 else root / "assets" / "field.rfield"

    reader = STEPControl_Reader()
    if reader.ReadFile(str(input_path)) != IFSelect_RetDone:
        raise RuntimeError(f"could not read STEP: {input_path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    BRepMesh_IncrementalMesh(shape, 0.06, False, 0.8, True)

    # The field uses 120 mm ore cubes. Collect their boxes before meshing so
    # every face of every cube gets a clear gray / red / blue alliance colour.
    ore_boxes = []
    solid_explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while solid_explorer.More():
        solid = TopoDS.Solid(solid_explorer.Current())
        box = Bnd_Box()
        BRepBndLib.Add_s(solid, box)
        if not box.IsVoid():
            x0, y0, z0 = box.GetXMin(), box.GetYMin(), box.GetZMin()
            x1, y1, z1 = box.GetXMax(), box.GetYMax(), box.GetZMax()
            sx = x1 - x0
            sy = y1 - y0
            sz = z1 - z0
            if (abs(sx - 120.0) < 0.5 and abs(sy - 120.0) < 0.5 and
                    abs(sz - 120.0) < 0.5):
                ore_boxes.append(
                    (x0 * 0.001, z0 * 0.001, y0 * 0.001,
                     x1 * 0.001, z1 * 0.001, y1 * 0.001)
                )
        solid_explorer.Next()

    vertices: list[tuple[float, float, float, float, float, float, int, int, int, int]] = []
    indices: list[int] = []
    min_v = [math.inf, math.inf, math.inf]
    max_v = [-math.inf, -math.inf, -math.inf]

    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        location = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, location)
        if tri is not None and tri.NbTriangles() > 0 and tri.NbNodes() > 0:
            reversed_face = face.Orientation() == TopAbs_REVERSED
            points = []
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(location.Transformation())
                # STEP is millimetres; simulator is metres. STEP is x/up/z,
                # simulator is x/y/up.
                points.append((p.X() * 0.001, p.Z() * 0.001, p.Y() * 0.001))

            triangles = []
            face_normal = [0.0, 0.0, 0.0]
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                ia, ib, ic = a - 1, b - 1, c - 1
                pa, pb, pc = points[ia], points[ib], points[ic]
                cx = (pa[0] + pb[0] + pc[0]) / 3.0
                cy = (pa[1] + pb[1] + pc[1]) / 3.0
                cz = (pa[2] + pb[2] + pc[2]) / 3.0
                ab = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
                ac = (pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2])
                nx = ab[1] * ac[2] - ab[2] * ac[1]
                ny = ab[2] * ac[0] - ab[0] * ac[2]
                nz = ab[0] * ac[1] - ab[1] * ac[0]
                ln = math.sqrt(nx * nx + ny * ny + nz * nz)
                if ln > 1e-12:
                    nx, ny, nz = nx / ln, ny / ln, nz / ln
                else:
                    nx, ny, nz = 0.0, 0.0, 1.0
                if reversed_face:
                    nx, ny, nz = -nx, -ny, -nz
                    triangles.append((ia, ic, ib))
                else:
                    triangles.append((ia, ib, ic))
                face_normal[0] += nx
                face_normal[1] += ny
                face_normal[2] += nz

            if not triangles:
                explorer.Next()
                face_index += 1
                continue

            vn = math.sqrt(sum(v * v for v in face_normal))
            if vn > 1e-12:
                face_normal = [v / vn for v in face_normal]
            else:
                face_normal = [0.0, 0.0, 1.0]
            center_z = sum(p[2] for p in points) / len(points)
            center_x = sum(p[0] for p in points) / len(points)
            center_y = sum(p[1] for p in points) / len(points)
            up = face_normal[2]
            near_center_platform = (abs(center_x) < 0.18 and abs(center_y) < 0.55 and
                                    center_z > 0.02)
            near_boundary = (abs(center_x) > 5.94 or abs(center_y) > 2.44)
            red_zone = center_x < -4.85 and center_y > 0.30 and center_z < 0.04
            blue_zone = center_x > 4.85 and center_y < -0.30 and center_z < 0.04

            is_ore = False
            for ox0, oy0, oz0, ox1, oy1, oz1 in ore_boxes:
                if (ox0 - 2e-4 <= center_x <= ox1 + 2e-4 and
                        oy0 - 2e-4 <= center_y <= oy1 + 2e-4 and
                        oz0 - 2e-4 <= center_z <= oz1 + 2e-4):
                    is_ore = True
                    break

            # SolidWorks face winding is not consistent after the STEP round-trip,
            # so use |n.z| for horizontal/sloped classification.
            horizontal = abs(up) > 0.65
            sloped = abs(up) > 0.15
            if is_ore:
                # 12 cm ore cubes: gray in the neutral centre, red on the red
                # side, blue on the blue side. The two cubes touching the long
                # edges are the red ores under the two bridges.
                near_long_edge = (abs(abs(center_y) - 2.44) < 0.08 and
                                  abs(center_x) < 0.20 and center_z < 0.15)
                if near_long_edge:
                    base = (0.62, 0.055, 0.09)
                elif abs(center_x) < 0.50:
                    base = (0.36, 0.38, 0.40)
                elif center_x < 0.0:
                    base = (0.62, 0.055, 0.09)
                else:
                    base = (0.035, 0.18, 0.70)
            elif horizontal:
                if center_z < 0.02:
                    if red_zone:
                        base = (0.64, 0.05, 0.12)
                    elif blue_zone:
                        base = (0.04, 0.22, 0.72)
                    else:
                        base = (0.10, 0.59, 0.64)
                elif near_center_platform:
                    base = (0.82, 0.68, 0.10)
                elif near_boundary:
                    base = (0.055, 0.047, 0.040)
                else:
                    base = (0.43, 0.32, 0.20)
            elif sloped:
                base = (0.38, 0.28, 0.18) if not near_center_platform else (0.72, 0.58, 0.08)
            else:
                if near_center_platform:
                    base = (0.05, 0.14, 0.48)
                elif near_boundary:
                    base = (0.055, 0.047, 0.040)
                else:
                    base = (0.175, 0.105, 0.060)

            color = hash_color(face_index + 1, base)
            rgba = tuple(int(clamp(c, 0.0, 1.0) * 255.0) for c in color) + (255,)
            base_index = len(vertices)
            for (px, py, pz), n, rgb in zip(points, [face_normal] * len(points), [rgba] * len(points)):
                # Mirror Y so the simulator view matches the supplied top-view
                # reference (red zone lower-left, blue zone upper-right).
                my = -py
                vertices.append((px, my, pz, n[0], -n[1], n[2], rgb[0], rgb[1], rgb[2], rgb[3]))
                min_v[0] = min(min_v[0], px)
                min_v[1] = min(min_v[1], my)
                min_v[2] = min(min_v[2], pz)
                max_v[0] = max(max_v[0], px)
                max_v[1] = max(max_v[1], my)
                max_v[2] = max(max_v[2], pz)
            for a, b, c in triangles:
                indices.extend((base_index + a, base_index + c, base_index + b))
        explorer.Next()
        face_index += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as f:
        f.write(b"RMFIELD1")
        f.write(struct.pack("<III", 1, len(vertices), len(indices)))
        f.write(struct.pack("<6f", *min_v, *max_v))
        f.write(struct.pack("<I", 0))
        for v in vertices:
            f.write(struct.pack("<6f4B", *v))
        f.write(struct.pack(f"<{len(indices)}I", *indices))

    print(
        f"wrote {output_path}  faces={face_index} vertices={len(vertices)} "
        f"triangles={len(indices)//3} ores={len(ore_boxes)} "
        f"bytes={output_path.stat().st_size} "
        f"bbox=({min_v[0]:.3f},{min_v[1]:.3f},{min_v[2]:.3f}).."
        f"({max_v[0]:.3f},{max_v[1]:.3f},{max_v[2]:.3f})"
    )


if __name__ == "__main__":
    main()
