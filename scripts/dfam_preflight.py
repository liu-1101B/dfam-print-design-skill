#!/usr/bin/env python3
"""DfAM geometric pre-screening for a printable mesh.

This tool answers GEOMETRY and MANUFACTURING questions only. It never claims a
part is strong enough: every mechanics check is reported as UNKNOWN until a
human supplies loads, allowables and an evidence grade.

Usage:
    python dfam_preflight.py MODEL.stl [--json] [--angle 45] [--no-edges]
    python dfam_preflight.py MODEL.3mf [--bed 256x256x256]
    python dfam_preflight.py MODEL.stl --weights support=.4,height=.2,contact=.2,ceiling=.2

Reads binary/ASCII STL and 3MF (core spec, nested component transforms).
Pure standard library: no numpy, no trimesh.
Exit code 0 for PASS/WARN, 1 for FAIL, 2 for usage error.
"""

import json
import math
import struct
import sys
import zipfile
import xml.etree.ElementTree as ET

TOOL = "dfam_preflight"
VERSION = "2.0"
SCOPE = "geometric pre-screening only; not a mechanical validation"

NS_3MF = "{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}"

BED_EPS = 0.05          # mm: faces this close above the bed count as first layer
FLAT_DEG = 5.0          # faces within 5 deg of horizontal are bridges/ceilings
DEFAULT_ANGLE = 45.0    # self-support angle from horizontal (a process parameter)
CONTACT_WARN = 0.15     # first-layer contact / bounding-box footprint
CONTACT_FAIL = 0.02
SUPPORT_WARN = 0.02     # support-needing projected area / downward projected area
SUPPORT_FAIL = 0.15
CEILING_WARN = 1.0      # mm^2 of near-horizontal unsupported ceiling

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
        out.append(a[3 * r] * b[9] + a[3 * r + 1] * b[10] + a[3 * r + 2] * b[11] + a[9 + r])
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
    """Return (boundary, nonmanifold, inconsistent) edge counts.

    Each undirected edge is packed as n | (s+128)<<8, where n is its triangle
    count and s the walking-direction balance: closed and consistently wound
    means n == 2 and s == 0 on every edge.
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


# Rotation that brings a chosen local axis DOWN to global -Z. Support ratio is
# invariant to spin about Z, so these six cover every axis-aligned pose.
POSES = [
    ("as-is", None),
    ("倒置", lambda v: (v[0], -v[1], -v[2])),
    ("侧躺·绕X", lambda v: (v[0], v[2], -v[1])),
    ("侧躺·绕X反", lambda v: (v[0], -v[2], v[1])),
    ("侧躺·绕Y", lambda v: (-v[2], v[1], v[0])),
    ("侧躺·绕Y反", lambda v: (v[2], v[1], -v[0])),
]

POSE_KEYS = ("support_ratio", "height_mm", "contact_mm2", "flat_ceiling_mm2")
POSE_BETTER = {"support_ratio": -1, "height_mm": -1, "contact_mm2": +1, "flat_ceiling_mm2": -1}


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
    rows = []
    for name, fn in POSES:
        moved = verts if fn is None else [fn(v) for v in verts]
        low = bbox(moved)[0][2]
        if low:
            moved = [(v[0], v[1], v[2] - low) for v in moved]
        st = face_stats(moved, tris, angle)
        rows.append(
            {
                "pose": name,
                "support_ratio": st["support_proj"] / st["down_proj"] if st["down_proj"] else 0.0,
                "support_mm2": st["support_proj"],
                "height_mm": bbox(moved)[1][2],
                "contact_mm2": st["contact"],
                "flat_ceiling_mm2": st["flat_ceiling"],
                "volume_mm3": st["volume"],
            }
        )
    flags, dominators = pareto_flags(rows)
    for row, flag, count in zip(rows, flags, dominators):
        row["pareto_optimal"] = flag
        row["dominated_by"] = count
    return rows


def pareto_flags(rows, keys=POSE_KEYS, better=POSE_BETTER):
    """Mark non-dominated rows and how many rows dominate each row.

    No weights are implied by this ordering: it only removes poses that are
    worse on every metric at once. Differences below a relative tolerance count
    as ties, so near-identical poses are not split apart by float noise.
    """
    flags, dominators = [], []
    for i, a in enumerate(rows):
        count = 0
        for j, b in enumerate(rows):
            if i == j:
                continue
            not_worse, strictly = True, False
            for k in keys:
                delta = better[k] * (b[k] - a[k])
                tol = 1e-6 * max(abs(a[k]), abs(b[k]), 1e-12)
                if delta < -tol:
                    not_worse = False
                    break
                if delta > tol:
                    strictly = True
            if not_worse and strictly:
                count += 1
        dominators.append(count)
        flags.append(count == 0)
    return flags, dominators


def weighted_scores(rows, weights):
    """Min-max normalise each metric, then apply explicit user weights."""
    norms = {}
    for key in POSE_KEYS:
        values = [r[key] for r in rows]
        lo, hi = min(values), max(values)
        norms[key] = (lo, hi)
    for row in rows:
        score = 0.0
        for key in POSE_KEYS:
            lo, hi = norms[key]
            if hi - lo <= 1e-12:
                frac = 1.0
            else:
                frac = (row[key] - lo) / (hi - lo)
                if POSE_BETTER[key] < 0:
                    frac = 1.0 - frac
            score += weights.get(key, 0.0) * frac
        row["weighted_score"] = round(score, 4)
    return {"method": "min-max over the six axis-aligned poses, lower is better", "weights": weights}


# ---------------------------------------------------------------- checks


def check(cid, name, state, value=None, limit=None, unit="", source="", note=""):
    return {
        "id": cid,
        "name": name,
        "state": state,
        "value": value,
        "limit": limit,
        "unit": unit,
        "source": source,
        "note": note,
    }


def audit_geometry(verts, tris, st, check_edges=True):
    boundary, nonmanifold, inconsistent = edge_health(tris) if check_edges else (0, 0, 0)
    out = []
    if check_edges:
        out.append(check(
            "G1", "网格封闭", "FAIL" if boundary or nonmanifold else "PASS",
            boundary + nonmanifold, 0, "edges",
            "边界边=1 次、非流形边>2 次都意味着壳不闭合",
            "先修水密，否则切片结果不可预测",
        ))
        out.append(check(
            "G2", "法向一致", "FAIL" if inconsistent else "PASS",
            inconsistent, 0, "edges",
            "闭合且绕向一致时每条边两次反向经过",
            "绕向不一致会让体积与内外判定都失效",
        ))
    else:
        out.append(check("G1", "网格封闭", "UNKNOWN", None, 0, "edges", "--no-edges", "本次未检查"))
        out.append(check("G2", "法向一致", "UNKNOWN", None, 0, "edges", "--no-edges", "本次未检查"))
    out.append(check(
        "G3", "无退化三角面", "WARN" if st["degenerate"] else "PASS",
        st["degenerate"], 0, "faces", "面积 < 1e-12 的三角面",
        "重复顶点或共线，建议清理",
    ))
    out.append(check(
        "G4", "体积为正", "FAIL" if st["volume"] <= 0 else "PASS",
        round(st["volume"], 3), 0, "mm3", "散度定理对闭合壳求和",
        "为负说明法向整体朝内",
    ))
    out.append(check(
        "G5", "无自相交", "UNKNOWN", None, None, "", "未实现",
        "壳相互穿插时应力与切片都不可预测，需在 CAD 里检查",
    ))
    return out


def audit_manufacturing(st, size, angle, bed):
    footprint = size[0] * size[1]
    ratio = st["support_proj"] / st["down_proj"] if st["down_proj"] else 0.0
    contact_ratio = st["contact"] / footprint if footprint else 0.0
    out = []
    if ratio > SUPPORT_FAIL:
        state, note = "FAIL", "改几何（见 geometry-patterns.md 卡片 2/5/6）或换姿态（见 poses）"
    elif ratio > SUPPORT_WARN:
        state, note = "WARN", "少量支撑；先比较改几何与换姿态的成本"
    else:
        state, note = "PASS", "当前姿态在给定自支撑角下基本免支撑"
    out.append(check(
        "M1", "悬垂需支撑率", state, round(ratio, 4), SUPPORT_FAIL, "ratio",
        "下向投影面积中超过 %.0f° 的部分 / 全部下向投影" % angle, note,
    ))
    out.append(check(
        "M2", "水平天花板/桥接", "WARN" if st["flat_ceiling"] > CEILING_WARN else "PASS",
        round(st["flat_ceiling"], 2), CEILING_WARN, "mm2",
        "与水平夹角 < %.0f° 的下向面" % FLAT_DEG,
        "桥接须两端锚固，否则改斜坡或拆件",
    ))
    if contact_ratio < CONTACT_FAIL:
        c_state, c_note = "FAIL", "首层接触过低，几乎必然脱板：加底面积/耳片或 brim"
    elif contact_ratio < CONTACT_WARN:
        c_state, c_note = "WARN", "首层接触偏低：加底面积/耳片或 brim"
    else:
        c_state, c_note = "PASS", "首层接触充足"
    out.append(check(
        "M3", "首层接触", c_state, round(contact_ratio, 4), CONTACT_WARN, "ratio",
        "床面朝下面积 / 包围盒底面积", c_note,
    ))
    if bed is None:
        out.append(check(
            "M4", "成形空间", "UNKNOWN", None, None, "mm", "未提供 --bed",
            "用 --bed 256x256x256 才能检查",
        ))
    else:
        over = [round(size[i] - bed[i], 2) for i in range(3)]
        fits = all(size[i] <= bed[i] + 1e-9 for i in range(3))
        out.append(check(
            "M4", "成形空间", "PASS" if fits else "FAIL", over, 0, "mm overflow",
            "包围盒 vs %.0fx%.0fx%.0f" % bed, "超出时拆件或换设备",
        ))
    out.append(check("M5", "最小壁厚", "UNKNOWN", None, None, "mm", "未实现",
                     "需体素化或射线法，当前不提供结论"))
    out.append(check("M6", "实际支撑体积与难拆区", "UNKNOWN", None, None, "mm3", "未实现",
                     "需切片器数据，几何预筛无法代替"))
    out.append(check("M7", "打印时间与材料消耗", "UNKNOWN", None, None, "", "未实现",
                     "需切片器估算"))
    out.append(check("M8", "尺寸精度与收缩", "UNKNOWN", None, None, "mm", "未实现",
                     "与设备、材料、尺寸相关，须实测首件"))
    return out


def audit_mechanics():
    items = [
        ("S1", "材料许用值", "缺同材料/同设备/同方向的 σ_xy、σ_z、E"),
        ("S2", "载荷与边界条件", "缺载荷大小/方向/作用点与支承方式"),
        ("S3", "强度（σ 与许用值）", "需按 mechanics.md 算危险截面"),
        ("S4", "刚度与变形", "需给出位移上限并计算挠度"),
        ("S5", "稳定性与屈曲", "长细比、薄壁板格需单独校核"),
        ("S6", "疲劳与冲击", "循环次数或冲击能量未知"),
        ("S7", "连接与孔边", "净截面、承压、边距需校核"),
        ("S8", "蠕变与温度", "工作温度与 Tg/HDT 的关系未知"),
    ]
    return [
        check(cid, name, "UNKNOWN", None, None, "", "需要人工输入", hint + "；见 references/mechanics.md")
        for cid, name, hint in items
    ]


ACTIONS = {
    "G1": "修水密：合并重复顶点、封闭开口、删除内部悬空壳",
    "G2": "统一法向：在 CAD 或切片器里重算法向/翻转内部壳",
    "G3": "清理退化面：删除零面积三角面与共线顶点",
    "G4": "整体翻转法向，或修掉内外颠倒的壳体",
    "G5": "在 CAD 里检查干涉/自相交，用布尔并集合并重叠实体",
    "M1": "改几何（倒角/斜坡/泪滴孔/拆件）或换姿态；两者成本要比较",
    "M2": "桥接两端加锚固平台，或改 45° 斜坡/拆件",
    "M3": "加底面积、加耳片、加 brim，或换姿态让大面贴床",
    "M4": "拆件、缩小尺寸或改用更大成形空间的设备",
    "M5": "用切片预览确认最薄处，必要时用体素工具测壁厚",
    "M6": "用切片器算支撑体积与难拆区，再决定改几何还是改姿态",
    "M7": "用切片器估算时间与材料，纳入成本比较",
    "M8": "打首件实测关键尺寸，据此修正补偿量",
}


def summarise(checks):
    states = [c["state"] for c in checks]
    if "FAIL" in states:
        return "FAIL"
    if "WARN" in states:
        return "WARN"
    if states and all(s == "UNKNOWN" for s in states):
        return "UNKNOWN"
    return "PASS"


# ---------------------------------------------------------------- report


def analyse(path, angle=DEFAULT_ANGLE, bed=None, weights=None, check_edges=True):
    if path.lower().endswith(".3mf"):
        verts, tris = load_3mf(path)
    else:
        verts, tris = load_stl(path)
    if not tris:
        raise ValueError("no triangles found")

    lo, hi = bbox(verts)
    size = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    st = face_stats(verts, tris, angle)

    geometry = audit_geometry(verts, tris, st, check_edges)
    manufacturing = audit_manufacturing(st, size, angle, bed)
    mechanics = audit_mechanics()
    poses = score_poses(verts, tris, angle)
    scoring = weighted_scores(poses, weights) if weights else None

    verdict = summarise(geometry + manufacturing)
    actions = []
    for c in geometry + manufacturing:
        if c["state"] in ("FAIL", "WARN") and c["id"] in ACTIONS:
            actions.append("%s %s：%s" % (c["id"], c["name"], ACTIONS[c["id"]]))
    if verdict != "FAIL":
        actions.append("几何预筛不能替代强度校核：承力件按 references/mechanics.md 补算或做 E3 试件")

    footprint = size[0] * size[1]
    return {
        "tool": TOOL,
        "version": VERSION,
        "scope": SCOPE,
        "file": path,
        "verdict": verdict,
        "verification": {
            "level": "E0",
            "geometric_screening": verdict,
            "mechanical_validation": "UNKNOWN",
            "statement": "几何预筛通过不等于结构验证通过；承力件需补算或实测。",
        },
        "geometry_status": {"state": summarise(geometry), "checks": geometry},
        "manufacturing_status": {"state": summarise(manufacturing), "checks": manufacturing},
        "mechanics_status": {
            "state": "UNKNOWN",
            "reason": "本工具不读取载荷、许用值与证据等级，力学项一律返回 UNKNOWN",
            "checks": mechanics,
        },
        "assumptions": [
            "自支撑角 %.0f°（工艺参数，应用时须按材料/层高/冷却实测）" % angle,
            "首层判定容差 %.2f mm" % BED_EPS,
            "首层接触阈值 %.0f%% 包围盒底面积" % (CONTACT_WARN * 100),
            "悬垂失败阈值 %.0f%% 下向投影" % (SUPPORT_FAIL * 100),
            "材料许用值未指定，全部力学结论为 UNKNOWN",
            "未做壁厚、自相交、支撑体积、时间估算",
        ],
        "unknowns": [c["id"] + " " + c["name"] for c in geometry + manufacturing + mechanics if c["state"] == "UNKNOWN"],
        "warnings": [c["id"] + " " + c["name"] + "：" + c["note"] for c in geometry + manufacturing if c["state"] == "WARN"],
        "recommended_actions": actions,
        "metrics": {
            "triangles": len(tris),
            "vertices": len(verts),
            "volume_mm3": round(st["volume"], 3),
            "area_mm2": round(st["area"], 2),
            "box_mm": [round(v, 3) for v in size],
            "footprint_mm2": round(footprint, 2),
            "first_layer_contact_mm2": round(st["contact"], 2),
            "first_layer_contact_ratio": round(st["contact"] / footprint, 4) if footprint else 0.0,
            "overhang_support_mm2": round(st["support_proj"], 2),
            "overhang_support_ratio": round(st["support_proj"] / st["down_proj"], 4) if st["down_proj"] else 0.0,
            "flat_ceiling_mm2": round(st["flat_ceiling"], 2),
            "self_supported_mm2": round(st["steep_proj"], 2),
        },
        "poses": poses,
        "pose_scoring": scoring,
    }


MARK = {"PASS": "ok ", "WARN": "warn", "FAIL": "FAIL", "UNKNOWN": "?  "}


def fmt_check_value(c):
    """Render one check's value for the human report."""
    if c["value"] is None:
        return ""
    value, unit = c["value"], c["unit"]
    if unit == "ratio":
        return " = %.1f%%" % (value * 100)
    if isinstance(value, list):
        return " = 超限 " + ", ".join("%g" % v for v in value) + (" " + unit if unit else "")
    text = " = %s" % value
    if unit:
        text += " " + unit.replace("mm2", "mm²").replace("mm3", "mm³")
    return text


def render(r):
    m = r["metrics"]
    out = ["DfAM 几何预筛 · %s   [%s %s]" % (r["file"], TOOL, r["version"])]
    out.append("范围   %s" % r["scope"])
    out.append(
        "网格   三角面 %d   顶点 %d   体积 %.2f cm³   表面积 %.0f mm²"
        % (m["triangles"], m["vertices"], m["volume_mm3"] / 1000.0, m["area_mm2"])
    )
    out.append(
        "包围盒 %.1f × %.1f × %.1f mm   首层接触 %.0f mm²（%.1f%%）   悬垂需支撑 %.1f%%"
        % (
            m["box_mm"][0], m["box_mm"][1], m["box_mm"][2],
            m["first_layer_contact_mm2"], m["first_layer_contact_ratio"] * 100,
            m["overhang_support_ratio"] * 100,
        )
    )
    for title, key in (("几何", "geometry_status"), ("制造", "manufacturing_status"), ("力学", "mechanics_status")):
        block = r[key]
        out.append("%s   %s" % (title, block["state"]))
        for c in block["checks"]:
            if c["state"] == "PASS" and title != "力学":
                continue
            out.append("   %s %s %s%s" % (MARK[c["state"]], c["id"], c["name"], fmt_check_value(c)))
    out.append("姿态（四项几何指标：支撑率↓ 高度↓ 首层接触↑ 天花板↓；未给硬约束与权重）")
    for p in r["poses"]:
        out.append(
            "   %-10s 支撑 %.1f%%  高 %.1f mm  首层 %.0f mm²  天花板 %.1f mm²   %s"
            % (
                p["pose"], p["support_ratio"] * 100, p["height_mm"],
                p["contact_mm2"], p["flat_ceiling_mm2"],
                "pareto" if p["pareto_optimal"] else "被 %d 个姿态支配" % p["dominated_by"],
            )
        )
    if r["pose_scoring"]:
        out.append("加权评分 %s" % json.dumps(r["pose_scoring"]["weights"], ensure_ascii=False))
        for p in sorted(r["poses"], key=lambda x: -x.get("weighted_score", 0)):
            out.append("   %-10s score=%.4f" % (p["pose"], p.get("weighted_score", 0)))
    out.append("结论   %s   （力学验证：UNKNOWN）" % r["verdict"])
    for msg in r["recommended_actions"]:
        out.append("   -> " + msg)
    return "\n".join(out)


def parse_bed(text):
    parts = text.lower().replace("*", "x").replace(",", "x").split("x")
    if len(parts) != 3:
        raise SystemExit("--bed expects three numbers like 256x256x256")
    return tuple(float(p) for p in parts)


def parse_weights(text):
    weights = {}
    for chunk in text.split(","):
        if not chunk.strip():
            continue
        key, _, value = chunk.partition("=")
        key = key.strip()
        aliases = {"support": "support_ratio", "height": "height_mm",
                   "contact": "contact_mm2", "ceiling": "flat_ceiling_mm2"}
        key = aliases.get(key, key)
        if key not in POSE_KEYS:
            raise SystemExit("unknown weight key: %s (use support/height/contact/ceiling)" % key)
        weights[key] = float(value)
    return weights


def main(argv):
    for stream in (sys.stdout, sys.stderr):
        try:  # Windows consoles default to GBK; the report is UTF-8
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, OSError):
            pass
    args, flags = [], set()
    angle = DEFAULT_ANGLE
    bed = None
    weights = None
    i = 1
    while i < len(argv):
        a = argv[i]
        if a == "--json":
            flags.add("json")
        elif a == "--no-edges":
            flags.add("no-edges")
        elif a == "--angle":
            i += 1
            angle = float(argv[i])
        elif a.startswith("--angle="):
            angle = float(a.split("=", 1)[1])
        elif a == "--bed":
            i += 1
            bed = parse_bed(argv[i])
        elif a.startswith("--bed="):
            bed = parse_bed(a.split("=", 1)[1])
        elif a == "--weights":
            i += 1
            weights = parse_weights(argv[i])
        elif a.startswith("--weights="):
            weights = parse_weights(a.split("=", 1)[1])
        elif a.startswith("--"):
            raise SystemExit("unknown flag: " + a)
        else:
            args.append(a)
        i += 1
    if not args:
        print("usage: python dfam_preflight.py MODEL.stl|MODEL.3mf [--json] [--angle 45] "
              "[--bed 256x256x256] [--weights support=.4,height=.2] [--no-edges]")
        return 2
    result = analyse(args[0], angle=angle, bed=bed, weights=weights,
                     check_edges="no-edges" not in flags)
    if "json" in flags:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        print(render(result))
    return 1 if result["verdict"] == "FAIL" else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
