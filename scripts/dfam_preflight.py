#!/usr/bin/env python3
"""DfAM preflight for a printable mesh: read-only geometry audit before slicing.

Answers the questions a DfAM review asks before the slicer does:
  * is the mesh closed and consistently oriented (will it slice at all)
  * how much downward surface needs support at a given self-support angle
  * how much bed contact the first layer gets
  * which of the six axis-aligned print orientations is cheapest to print

Usage:
    python dfam_preflight.py MODEL.stl [--json] [--angle 45] [--no-edges]
    python dfam_preflight.py MODEL.3mf [--json] [--angle 45] [--no-edges]

Reads binary/ASCII STL and 3MF (core spec, nested component transforms).
Pure standard library: no numpy, no trimesh.
Exit code 0 for PASS/WARN, 1 for FAIL, so it can gate a build step.
"""

import json
import math
import struct
import sys
import zipfile
import xml.etree.ElementTree as ET

NS_3MF = "{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}"

BED_EPS = 0.05          # mm: faces this close above the bed count as first layer
FLAT_DEG = 5.0          # faces within 5 deg of horizontal are bridges/ceilings
DEFAULT_ANGLE = 45.0    # self-support angle measured from horizontal
WARN_CONTACT_RATIO = 0.15   # matches the 15% first-layer rule in SKILL.md

# ---------------------------------------------------------------- loading


def load_stl(path):
    """Return (verts, tris) from a binary or ASCII STL file."""
    with open(path, "rb") as fh:
        blob = fh.read()
    if len(blob) < 84:
        raise ValueError("file is too small to be an STL")
    count = struct.unpack_from("<I", blob, 80)[0]
    if len(blob) == 84 + 50 * count:
        verts, tris = [], []
        index = {}
        unpack = struct.Struct("<12fH").unpack_from
        for i in range(count):
            rec = unpack(blob, 84 + 50 * i)
            tri = []
            for x, y, z in ((rec[3], rec[4], rec[5]), (rec[6], rec[7], rec[8]), (rec[9], rec[10], rec[11])):
                key = (round(x, 6), round(y, 6), round(z, 6))
                vi = index.get(key)
                if vi is None:
                    vi = len(verts)
                    index[key] = vi
                    verts.append(key)
                tri.append(vi)
            tris.append((tri[0], tri[1], tri[2]))
        return verts, tris
    return load_ascii_stl(blob.decode("utf-8", "replace"))


def load_ascii_stl(text):
    if "facet" not in text:
        raise ValueError("not an STL: binary size mismatch and no ASCII facets")
    verts, tris, index, tri = [], [], {}, []
    for raw in text.splitlines():
        parts = raw.split()
        if not parts or parts[0] != "vertex" or len(parts) < 4:
            continue
        key = (float(parts[1]), float(parts[2]), float(parts[3]))
        vi = index.get(key)
        if vi is None:
            vi = len(verts)
            index[key] = vi
            verts.append(key)
        tri.append(vi)
        if len(tri) == 3:
            tris.append((tri[0], tri[1], tri[2]))
            tri = []
    return verts, tris


def load_3mf(path):
    """Return (verts, tris) merged over every build item, transforms applied."""
    with zipfile.ZipFile(path) as zf:
        root = ET.fromstring(zf.read("3D/3dmodel.model"))

    meshes, components = {}, {}
    for obj in root.iter(NS_3MF + "object"):
        oid = obj.get("id")
        mesh = obj.find(NS_3MF + "mesh")
        if mesh is not None:
            vs = [
                (float(v.get("x")), float(v.get("y")), float(v.get("z")))
                for v in mesh.iter(NS_3MF + "vertex")
            ]
            ts = [
                (int(t.get("v1")), int(t.get("v2")), int(t.get("v3")))
                for t in mesh.iter(NS_3MF + "triangle")
            ]
            meshes[oid] = (vs, ts)
        comps = obj.find(NS_3MF + "components")
        if comps is not None:
            components[oid] = [
                (c.get("objectid"), parse_transform(c.get("transform")))
                for c in comps.findall(NS_3MF + "component")
            ]

    verts, tris = [], []

    def add(vs, ts, matrix):
        base = len(verts)
        for v in vs:
            verts.append(apply_matrix(matrix, v))
        for a, b, c in ts:
            tris.append((base + a, base + b, base + c))

    def add_object(oid, applied, depth=0, seen=()):
        if depth > 8 or oid in seen:
            return
        entry = meshes.get(oid)
        if entry is not None:
            add(entry[0], entry[1], applied)
        for child, child_matrix in components.get(oid, ()):
            add_object(child, compose(child_matrix, applied), depth + 1, seen + (oid,))

    build = root.find(NS_3MF + "build")
    items = list(build.findall(NS_3MF + "item")) if build is not None else []
    if not items:  # no build plate: fall back to every mesh object
        items = [ET.Element(NS_3MF + "item", {"objectid": k}) for k in meshes]
    for item in items:
        add_object(item.get("objectid"), parse_transform(item.get("transform")))
    return verts, tris


def parse_transform(text):
    if not text:
        return None
    nums = [float(n) for n in text.replace(",", " ").split()]
    return nums if len(nums) == 12 else None


def apply_matrix(m, v):
    if m is None:
        return v
    x, y, z = v
    return (
        m[0] * x + m[3] * y + m[6] * z + m[9],
        m[1] * x + m[4] * y + m[7] * z + m[10],
        m[2] * x + m[5] * y + m[8] * z + m[11],
    )


def compose(a, b):
    """Row-vector 3MF transform product a*b: apply a first, then b."""
    if a is None:
        return b
    if b is None:
        return a
    out = []
    for r in range(3):
        for c in range(3):
            out.append(a[3 * r] * b[c] + a[3 * r + 1] * b[3 + c] + a[3 * r + 2] * b[6 + c])
    for r in range(3):
        out.append(
            a[3 * r] * b[9] + a[3 * r + 1] * b[10] + a[3 * r + 2] * b[11] + a[9 + r]
        )
    return out


# ---------------------------------------------------------------- geometry


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def bbox(verts):
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def edge_health(tris):
    """Return (boundary_edges, nonmanifold_edges, inconsistent_edges).

    Every undirected edge is packed as n | (s+128)<<8, where n is its triangle
    count and s the walking-direction balance: a closed, consistently wound mesh
    has n == 2 and s == 0 on every edge.
    """
    counts = {}
    for a, b, c in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            packed = counts.get(key, 128 << 8)   # n = 0, s = 0
            n = packed & 0xFF
            s = (packed >> 8) - 128
            s += 1 if u < v else -1
            counts[key] = (n + 1) | ((s + 128) << 8)
    boundary = nonmanifold = inconsistent = 0
    for packed in counts.values():
        n = packed & 0xFF
        if n == 1:
            boundary += 1
        elif n > 2:
            nonmanifold += 1
        elif (packed >> 8) - 128 != 0:
            inconsistent += 1
    return boundary, nonmanifold, inconsistent


# Rotation that brings a chosen local axis DOWN to global -Z. Each entry maps
# local (x, y, z) to the printed frame; support ratio is invariant to spin
# about Z, so these six cover every axis-aligned pose.
POSES = [
    ("as-is", None),
    ("倒置", lambda v: (v[0], -v[1], -v[2])),
    ("侧躺·绕X", lambda v: (v[0], v[2], -v[1])),
    ("侧躺·绕X反", lambda v: (v[0], -v[2], v[1])),
    ("侧躺·绕Y", lambda v: (-v[2], v[1], v[0])),
    ("侧躺·绕Y反", lambda v: (v[2], v[1], -v[0])),
]


def face_stats(verts, tris, angle):
    """Accumulate volume, area, support need, bed contact and bridge area."""
    zmin = bbox(verts)[0][2]
    limit = math.cos(math.radians(angle))
    flat = math.cos(math.radians(FLAT_DEG))
    vol = area_total = down_proj = support = contact = ceiling = steep = 0.0
    degenerate = 0
    for a, b, c in tris:
        pa, pb, pc = verts[a], verts[b], verts[c]
        vol += dot(pa, cross(pb, pc)) / 6.0
        n = cross(sub(pb, pa), sub(pc, pa))
        mag = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])
        if mag < 1e-12:
            degenerate += 1
            continue
        area = 0.5 * mag
        nz = n[2] / mag
        area_total += area
        if nz >= 0.0:
            continue
        proj = -area * nz
        down_proj += proj
        if nz < -0.99 and max(pa[2], pb[2], pc[2]) <= zmin + BED_EPS:
            contact += proj
            continue
        if -nz > limit:                    # shallower than the self-support angle
            support += proj
            if -nz > flat:
                ceiling += proj
        else:
            steep += proj
    return {
        "volume": vol,
        "area": area_total,
        "degenerate": degenerate,
        "down_proj": down_proj,
        "support_proj": support,
        "contact": contact,
        "flat_ceiling": ceiling,
        "steep_proj": steep,
    }


def score_poses(verts, tris, angle):
    scored = []
    for name, fn in POSES:
        moved = verts if fn is None else [fn(v) for v in verts]
        low = bbox(moved)[0][2]
        if low:
            moved = [(v[0], v[1], v[2] - low) for v in moved]
        st = face_stats(moved, tris, angle)
        ratio = st["support_proj"] / st["down_proj"] if st["down_proj"] else 0.0
        hi = bbox(moved)[1][2]
        scored.append(
            {
                "pose": name,
                "support_ratio": ratio,
                "support_mm2": st["support_proj"],
                "height_mm": hi,
                "flat_ceiling_mm2": st["flat_ceiling"],
            }
        )
    scored.sort(key=lambda r: (round(r["support_ratio"], 4), round(r["height_mm"], 2)))
    return scored


# ---------------------------------------------------------------- report


def analyse(path, angle=DEFAULT_ANGLE, check_edges=True):
    if path.lower().endswith(".3mf"):
        verts, tris = load_3mf(path)
    else:
        verts, tris = load_stl(path)
    if not tris:
        raise ValueError("no triangles found")

    lo, hi = bbox(verts)
    st = face_stats(verts, tris, angle)
    boundary, nonmanifold, inconsistent = edge_health(tris) if check_edges else (0, 0, 0)

    size = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    footprint = size[0] * size[1]
    ratio = st["support_proj"] / st["down_proj"] if st["down_proj"] else 0.0
    contact_ratio = st["contact"] / footprint if footprint else 0.0

    fails, warns = [], []
    if st["volume"] < 0:
        fails.append("体积为负：法向整体朝内，切片器会当成空腔——先在 CAD 里统一法向")
    if boundary or nonmanifold:
        fails.append(
            "网格不封闭：边界边 %d 条、非流形边 %d 条——先修水密，否则切片不可预测"
            % (boundary, nonmanifold)
        )
    if inconsistent:
        fails.append(
            "面朝向不一致：%d 条边两侧同向，体积与内外判定都不可信——先统一法向" % inconsistent
        )
    if st["degenerate"]:
        warns.append("退化三角面 %d 个（面积≈0）：多为重复顶点或共线，建议清理" % st["degenerate"])
    if ratio > 0.15:
        fails.append(
            "悬垂超过 %.0f° 的面占下向投影 %.1f%%：必须改几何（45° 倒角/斜坡/泪滴孔）或接受大量支撑"
            % (angle, ratio * 100)
        )
    elif ratio > 0.02:
        warns.append(
            "悬垂超过 %.0f° 的面占下向投影 %.1f%%（%.1f mm²）：少量支撑；能改成 45° 斜面就改"
            % (angle, ratio * 100, st["support_proj"])
        )
    if st["flat_ceiling"] > 1.0:
        warns.append(
            "水平天花板 %.1f mm²：桥接必须两端锚固，否则改斜坡或拆件" % st["flat_ceiling"]
        )
    if contact_ratio < WARN_CONTACT_RATIO:
        warns.append(
            "首层接触仅 %.0f mm²（占包围盒 %.1f%%）：加底面积/耳片或 brim，否则易脱板"
            % (st["contact"], contact_ratio * 100)
        )

    verdict = "FAIL" if fails else ("WARN" if warns else "PASS")
    return {
        "file": path,
        "verdict": verdict,
        "fails": fails,
        "warns": warns,
        "mesh": {
            "vertices": len(verts),
            "triangles": len(tris),
            "watertight": (boundary == 0 and nonmanifold == 0) if check_edges else None,
            "consistent": (inconsistent == 0) if check_edges else None,
            "boundary_edges": boundary,
            "nonmanifold_edges": nonmanifold,
            "inconsistent_edges": inconsistent,
            "degenerate": st["degenerate"],
            "volume_mm3": st["volume"],
            "area_mm2": st["area"],
        },
        "box": {"size": size, "min": lo, "max": hi, "footprint_mm2": footprint},
        "first_layer": {"contact_mm2": st["contact"], "contact_ratio": contact_ratio},
        "overhang": {
            "self_support_deg": angle,
            "support_mm2": st["support_proj"],
            "support_ratio": ratio,
            "down_proj_mm2": st["down_proj"],
            "flat_ceiling_mm2": st["flat_ceiling"],
            "self_supported_mm2": st["steep_proj"],
        },
        "poses": score_poses(verts, tris, angle),
    }


def render(r):
    m, b, o = r["mesh"], r["box"], r["overhang"]
    sx, sy, sz = b["size"]
    out = ["DfAM 预检 · %s" % r["file"]]
    out.append(
        "网格   三角面 %d   顶点 %d   水密 %s   法向一致 %s   体积 %.2f cm³   表面积 %.0f mm²"
        % (
            m["triangles"],
            m["vertices"],
            "—" if m["watertight"] is None else ("是" if m["watertight"] else "否"),
            "—" if m["consistent"] is None else ("是" if m["consistent"] else "否"),
            m["volume_mm3"] / 1000.0,
            m["area_mm2"],
        )
    )
    out.append(
        "包围盒 %.1f × %.1f × %.1f mm   首层接触 %.0f mm²（占包围盒 %.1f%%）"
        % (sx, sy, sz, r["first_layer"]["contact_mm2"], r["first_layer"]["contact_ratio"] * 100)
    )
    out.append(
        "悬垂   需支撑 %.1f%%（%.1f / %.1f mm²）   水平天花板 %.1f mm²   自支撑面 %.0f mm²"
        % (
            o["support_ratio"] * 100,
            o["support_mm2"],
            o["down_proj_mm2"],
            o["flat_ceiling_mm2"],
            o["self_supported_mm2"],
        )
    )
    best = [p for p in r["poses"] if p["pose"] != "as-is"]
    if best and best[0]["support_ratio"] < o["support_ratio"] - 0.005:
        out.append(
            "姿态   当前 %.1f%% → 换成「%s」可降到 %.1f%%（高 %.1f mm）"
            % (o["support_ratio"] * 100, best[0]["pose"], best[0]["support_ratio"] * 100, best[0]["height_mm"])
        )
    else:
        out.append("姿态   当前已是最省支撑的轴向姿态（%.1f%%）" % (o["support_ratio"] * 100))
    out.append("结论   %s" % r["verdict"])
    for msg in r["fails"]:
        out.append("   x " + msg)
    for msg in r["warns"]:
        out.append("   ! " + msg)
    return "\n".join(out)


def main(argv):
    for stream in (sys.stdout, sys.stderr):
        try:  # Windows consoles default to GBK; the report is UTF-8
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, OSError):
            pass
    args, flags = [], set()
    i = 1
    while i < len(argv):
        a = argv[i]
        if a == "--json":
            flags.add("json")
        elif a == "--no-edges":
            flags.add("no-edges")
        elif a == "--angle":
            i += 1
            flags.add("angle=" + argv[i])
        elif a.startswith("--angle="):
            flags.add("angle=" + a.split("=", 1)[1])
        elif a.startswith("--"):
            raise SystemExit("unknown flag: " + a)
        else:
            args.append(a)
        i += 1
    if not args:
        print("usage: python dfam_preflight.py MODEL.stl|MODEL.3mf [--json] [--angle 45] [--no-edges]")
        return 2
    angle = DEFAULT_ANGLE
    for f in flags:
        if f.startswith("angle="):
            angle = float(f.split("=", 1)[1])
    result = analyse(args[0], angle=angle, check_edges="no-edges" not in flags)
    if "json" in flags:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        print(render(result))
    return 1 if result["verdict"] == "FAIL" else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
