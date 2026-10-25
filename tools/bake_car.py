"""Build the local car render mesh from the supplied SolidWorks parts.

Runtime only loads assets/car.rmesh. This script uses the open-source `sldkit`
reader and fast-simplification at asset-build time.

Usage: python tools/bake_car.py
"""
import math
import struct
import zipfile
from pathlib import Path

import fast_simplification
import numpy as np
import sldkit

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "assets" / "car.rmesh"


def locate_car_root():
    """Find the local SLDPRT tree, extracting the supplied ZIP if needed."""
    existing = list(ROOT.glob("assets/car_cad/**/*.SLDASM"))
    if existing:
        return existing[0].parent
    archives = sorted(ROOT.glob("*.zip"), key=lambda p: p.stat().st_size, reverse=True)
    if not archives:
        raise FileNotFoundError("car CAD ZIP not found")
    target = ROOT / "build" / "car_cad"
    target.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archives[0]) as zf:
        zf.extractall(target)
    assemblies = list(target.rglob("*.SLDASM"))
    if not assemblies:
        raise FileNotFoundError("no SLDASM in car CAD ZIP")
    return assemblies[0].parent


CAR_ROOT = locate_car_root()


_PART_CACHE = None


def parts_by_signature():
    """Identify parts by their local bounding box.

    The CAD archive stores Chinese filenames in a legacy code page, so matching
    literal filenames is unreliable. Geometry signatures are language-neutral.
    """
    global _PART_CACHE
    if _PART_CACHE is not None:
        return _PART_CACHE
    rows = []
    for path in CAR_ROOT.rglob("*.SLDPRT"):
        try:
            mesh = sldkit.decode_display_mesh_file(path)
        except Exception:
            continue
        if not mesh.positions:
            continue
        verts = np.frombuffer(mesh.positions, dtype=np.float32).reshape(-1, 3)
        size = verts.max(axis=0) - verts.min(axis=0)
        rows.append((path, tuple(float(v) for v in size), len(mesh.indices) // 12))
    rows.sort(key=lambda r: str(r[0]))
    _PART_CACHE = rows
    return rows


def find_size(size, tolerance=3.0):
    hits = []
    for path, part_size, tris in parts_by_signature():
        if all(abs(part_size[i] - size[i]) <= tolerance for i in range(3)):
            hits.append(path)
    if not hits:
        raise FileNotFoundError(f"no part with size {size}")
    return hits


def load_part(path, target_tris):
    mesh = sldkit.decode_display_mesh_file(path)
    verts = np.frombuffer(mesh.positions, dtype=np.float32).reshape(-1, 3).astype(np.float64)
    faces = np.frombuffer(mesh.indices, dtype=np.uint32).reshape(-1, 3).astype(np.int32)
    if len(faces) > target_tris:
        reduction = max(0.0, min(0.98, 1.0 - target_tris / len(faces)))
        verts, faces = fast_simplification.simplify(verts, faces, target_reduction=reduction)
        faces = faces.astype(np.int32)
    # mm -> m
    verts = verts * 0.001
    center = (verts.min(axis=0) + verts.max(axis=0)) * 0.5
    verts = verts - center
    return verts, faces


def rot_x(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def add_part(tris, path, target_tris, rotation, translation, color):
    verts, faces = load_part(path, target_tris)
    verts = verts @ rotation.T + np.asarray(translation, dtype=np.float64)
    # Flat shaded triangle soup.
    p0 = verts[faces[:, 0]]
    p1 = verts[faces[:, 1]]
    p2 = verts[faces[:, 2]]
    n = np.cross(p1 - p0, p2 - p0)
    ln = np.linalg.norm(n, axis=1)
    ln[ln < 1e-12] = 1.0
    n = n / ln[:, None]
    rgba = tuple(int(max(0.0, min(1.0, c)) * 255) for c in color) + (255,)
    for i in range(len(faces)):
        tris.append(
            (
                tuple(p0[i]) + tuple(n[i]) + rgba,
                tuple(p1[i]) + tuple(n[i]) + rgba,
                tuple(p2[i]) + tuple(n[i]) + rgba,
            )
        )


def main():
    tris = []
    # Local SolidWorks parts. Four 580 mm tubes make the square frame; the
    # upper rails/cross braces reproduce the white frame in the local CAD.
    frame = (0.80, 0.82, 0.82)
    tube = find_size((20.0, 20.0, 580.0))[0]
    for y in (-0.28, 0.28):
        add_part(tris, tube, 1600, rot_y(90), (0.0, y, 0.072), frame)
        add_part(tris, tube, 1600, rot_y(90), (0.0, y, 0.148), frame)
    for x in (-0.28, 0.28):
        add_part(tris, tube, 1600, rot_x(-90), (x, 0.0, 0.072), frame)
        add_part(tris, tube, 1600, rot_x(-90), (x, 0.0, 0.148), frame)

    plate = find_size((340.0, 250.0, 5.0))[0]
    add_part(tris, plate, 4000, np.eye(3), (0.0, 0.0, 0.047), (0.15, 0.25, 0.52))

    wheels = find_size((152.2, 152.2, 45.5), tolerance=2.0)
    if len(wheels) < 2:
        raise RuntimeError("expected left/right mecanum wheel parts")
    for sx in (-1, 1):
        for sy in (-1, 1):
            wheel = wheels[0] if sy > 0 else wheels[1]
            add_part(
                tris,
                wheel,
                6000,
                rot_x(90 if sy > 0 else -90),
                (sx * 0.21, sy * 0.26, 0.076),
                (0.28, 0.29, 0.32),
            )

    motor = find_size((41.9, 98.3, 47.0), tolerance=2.0)[0]
    for sx in (-1, 1):
        for sy in (-1, 1):
            add_part(
                tris,
                motor,
                1200,
                rot_x(-90),
                (sx * 0.20, sy * 0.20, 0.115),
                (0.45, 0.46, 0.48),
            )

    battery = find_size((69.4, 39.5, 103.5), tolerance=2.0)[0]
    add_part(tris, battery, 1800, np.eye(3), (0.0, 0.015, 0.112), (0.12, 0.32, 0.75))

    board = find_size((60.0, 20.8, 45.0), tolerance=2.0)[0]
    add_part(tris, board, 1800, np.eye(3), (0.15, -0.10, 0.150), (0.08, 0.50, 0.20))

    # Black speed controllers and the main electronics box visible on top.
    esc = find_size((293.4, 11.5, 26.1), tolerance=2.0)[0]
    for y in (-0.16, 0.16):
        add_part(tris, esc, 900, np.eye(3), (0.0, y, 0.152), (0.10, 0.10, 0.11))
    main_box = find_size((101.0, 58.0, 36.0), tolerance=2.0)[0]
    add_part(tris, main_box, 1200, np.eye(3), (-0.16, 0.02, 0.165), (0.08, 0.08, 0.09))

    if not tris:
        raise RuntimeError("no local car geometry")

    positions = np.array([[v[0], v[1], v[2]] for tri in tris for v in tri], dtype=np.float64)
    bmin = positions.min(axis=0)
    bmax = positions.max(axis=0)
    print("triangles", len(tris), "bbox", bmin, bmax)

    # Put the wheel contact (lowest point) at z=0 and center X/Y.
    cx = (bmin[0] + bmax[0]) * 0.5
    cy = (bmin[1] + bmax[1]) * 0.5
    z0 = bmin[2]
    shifted = []
    for tri in tris:
        shifted.append(
            tuple((v[0] - cx, v[1] - cy, v[2] - z0, *v[3:]) for v in tri)
        )

    batches = []
    batch = []
    for tri in shifted:
        if len(batch) + 3 > 60000:
            batches.append(batch)
            batch = []
        batch.extend(tri)
    if batch:
        batches.append(batch)

    xs = [v[0] for tri in shifted for v in tri]
    ys = [v[1] for tri in shifted for v in tri]
    zs = [v[2] for tri in shifted for v in tri]
    with OUT.open("wb") as f:
        f.write(b"RMCAR001")
        f.write(struct.pack("<II", 1, len(batches)))
        f.write(struct.pack("<6f", min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)))
        f.write(struct.pack("<I", 0))
        for batch in batches:
            f.write(struct.pack("<II", len(batch), len(batch)))
            for v in batch:
                f.write(struct.pack("<6f4B", *v))
            f.write(struct.pack(f"<{len(batch)}H", *range(len(batch))))
    print("wrote", OUT, OUT.stat().st_size, "batches", len(batches))
    print("centered bbox", (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)))


if __name__ == "__main__":
    main()
