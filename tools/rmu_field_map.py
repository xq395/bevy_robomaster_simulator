"""Bake a top-down 2D base map of the field from assets/GROUND.glb for the terminal's tactical map.

Field coords match RMU RobotPosition / MapClick: x 0..28 m from the red side, y 0..15 m.
    python tools/rmu_field_map.py            -> tools/rmu_field.png (+ prints the height range)
"""
from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
FIELD_W, FIELD_H = 28.0, 15.0
COMP = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}
NCOMP = {"SCALAR": 1, "VEC3": 3}


def load_glb(path: Path):
    b = path.read_bytes()
    jlen = struct.unpack_from("<I", b, 12)[0]
    j = json.loads(b[20:20 + jlen])
    bin_off = 20 + jlen + 8
    return j, b[bin_off:]


def accessor(j, blob, idx):
    a = j["accessors"][idx]
    v = j["bufferViews"][a["bufferView"]]
    dt, n = COMP[a["componentType"]], NCOMP[a["type"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    stride = v.get("byteStride", 0)
    item = np.dtype(dt).itemsize * n
    if stride and stride != item:
        raw = np.frombuffer(blob, np.uint8, count=stride * a["count"], offset=off).reshape(-1, stride)[:, :item]
        return raw.copy().view(dt).reshape(a["count"], n)
    return np.frombuffer(blob, dt, count=a["count"] * n, offset=off).reshape(a["count"], n)


def quat_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def node_matrix(nd):
    m = np.eye(4)
    s = np.diag(nd.get("scale", [1, 1, 1]))
    m[:3, :3] = quat_mat(nd.get("rotation", [0, 0, 0, 1])) @ s
    m[:3, 3] = nd.get("translation", [0, 0, 0])
    return m


def triangles(j, blob, node_name="GROUND_DENSE"):
    """World-space triangles (N,3,3) of one node's mesh."""
    nd = next(n for n in j["nodes"] if n.get("name") == node_name)
    m = node_matrix(nd)
    out = []
    for prim in j["meshes"][nd["mesh"]]["primitives"]:
        pos = accessor(j, blob, prim["attributes"]["POSITION"]).astype(np.float64)
        pos = pos @ m[:3, :3].T + m[:3, 3]
        idx = accessor(j, blob, prim["indices"]).reshape(-1).astype(np.int64) if "indices" in prim else np.arange(len(pos))
        out.append(pos[idx.reshape(-1, 3)])
    return np.concatenate(out)


def rasterize(tris, res=0.05):
    """Max-height grid over the field; each triangle is sampled densely enough to cover its cells."""
    gw, gh = int(FIELD_W / res), int(FIELD_H / res)
    grid = np.full((gh, gw), np.nan)
    # glTF: x across the field (red at -x), y up, z toward the camera -> field y = 7.5 - z.
    fx = tris[..., 0] + FIELD_W / 2
    fy = FIELD_H / 2 - tris[..., 2]
    h = tris[..., 1]
    span = np.maximum(np.ptp(fx, axis=1), np.ptp(fy, axis=1))
    steps = np.ceil(span / res * 1.5).astype(int).clip(1)
    flatg = np.full(gw * gh, -1e9)
    for t in np.nonzero(steps > 64)[0]:  # large triangles: exact raster over their bounding box
        x0, x1 = max(int(fx[t].min() / res), 0), min(int(fx[t].max() / res) + 1, gw)
        y0, y1 = max(int(fy[t].min() / res), 0), min(int(fy[t].max() / res) + 1, gh)
        if x0 >= x1 or y0 >= y1:
            continue
        cx, cy = np.meshgrid((np.arange(x0, x1) + 0.5) * res, (np.arange(y0, y1) + 0.5) * res)
        (ax, bx, qx), (ay, by, qy) = fx[t], fy[t]
        den = (by - qy) * (ax - qx) + (qx - bx) * (ay - qy)
        if abs(den) < 1e-12:
            continue
        l0 = ((by - qy) * (cx - qx) + (qx - bx) * (cy - qy)) / den
        l1 = ((qy - ay) * (cx - qx) + (ax - qx) * (cy - qy)) / den
        l2 = 1 - l0 - l1
        inside = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        hh = l0 * h[t, 0] + l1 * h[t, 1] + l2 * h[t, 2]
        iy, ix = np.nonzero(inside)
        np.maximum.at(flatg, (iy + y0) * gw + ix + x0, hh[inside])
    grid = np.where(flatg == -1e9, np.nan, flatg).reshape(gh, gw)
    for k in np.unique(steps[steps <= 64]):
        sel = steps == k
        u, v = np.meshgrid(np.linspace(0, 1, k + 1), np.linspace(0, 1, k + 1))
        keep = (u + v) <= 1
        u, v = u[keep], v[keep]
        w0 = 1 - u - v
        px = fx[sel, 0:1] * w0 + fx[sel, 1:2] * u + fx[sel, 2:3] * v
        py = fy[sel, 0:1] * w0 + fy[sel, 1:2] * u + fy[sel, 2:3] * v
        ph = h[sel, 0:1] * w0 + h[sel, 1:2] * u + h[sel, 2:3] * v
        ix = (px / res).astype(int).ravel()
        iy = (py / res).astype(int).ravel()
        ph = ph.ravel()
        ok = (ix >= 0) & (ix < gw) & (iy >= 0) & (iy < gh)
        flat = iy[ok] * gw + ix[ok]
        cur = np.nan_to_num(grid.ravel(), nan=-1e9)
        np.maximum.at(cur, flat, ph[ok])
        cur[cur == -1e9] = np.nan
        grid = cur.reshape(gh, gw)
    return grid


def shade(grid, res):
    """Height bands + hillshade + edge lines so walls/steps read clearly on a dark UI."""
    g = np.nan_to_num(grid, nan=np.nanmin(grid))
    floor = np.percentile(g, 20)
    rel = np.clip(g - floor, 0, None)
    gy, gx = np.gradient(g, res)
    slope = np.hypot(gx, gy)
    light = np.clip(0.5 + (-gx * 0.6 + gy * 0.6) * 0.15, 0.2, 1.0)
    top = np.clip(rel / 0.9, 0, 1) ** 0.7
    base = np.stack([10 + 50 * top, 20 + 70 * top, 36 + 90 * top], -1) * (0.7 + 0.3 * light[..., None])
    wall = slope > 3.0
    base[wall] = (120, 150, 185)
    base[np.isnan(grid)] = (6, 12, 22)
    return np.flipud(base).clip(0, 255).astype(np.uint8)  # row 0 = field y max (top of canvas)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--glb", type=Path, default=ROOT / "assets" / "GROUND.glb")
    ap.add_argument("--out", type=Path, default=ROOT / "tools" / "rmu_field.png")
    ap.add_argument("--res", type=float, default=0.025, help="metres per pixel")
    a = ap.parse_args(argv)
    from PIL import Image

    j, blob = load_glb(a.glb)
    tris = triangles(j, blob)
    grid = rasterize(tris, a.res)
    img = Image.fromarray(shade(grid, a.res))
    img.save(a.out, optimize=True)
    print(f"{len(tris)} 三角形 -> {img.size[0]}x{img.size[1]} {a.out} ({a.out.stat().st_size // 1024} KB)，"
          f"高度 {np.nanmin(grid):.2f}..{np.nanmax(grid):.2f} m，空洞 {np.isnan(grid).mean() * 100:.1f}%")


if __name__ == "__main__":
    main()
