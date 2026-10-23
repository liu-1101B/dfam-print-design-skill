"""Regression tests for the standard-library DfAM preflight tool.

These tests cover deterministic geometry fixtures and command-line output. They
do not assert that a printed part is mechanically safe.
"""
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "dfam_preflight.py"

spec = importlib.util.spec_from_file_location("dfam_preflight", SCRIPT)
dfam = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(dfam)


def write_unit_cube_stl(path):
    """Write a closed, consistently wound 1 mm cube as binary STL."""
    p000 = (0.0, 0.0, 0.0)
    p001 = (0.0, 0.0, 1.0)
    p010 = (0.0, 1.0, 0.0)
    p011 = (0.0, 1.0, 1.0)
    p100 = (1.0, 0.0, 0.0)
    p101 = (1.0, 0.0, 1.0)
    p110 = (1.0, 1.0, 0.0)
    p111 = (1.0, 1.0, 1.0)
    triangles = [
        # Bottom (-Z), top (+Z)
        (p000, p010, p110), (p000, p110, p100),
        (p001, p101, p111), (p001, p111, p011),
        # X-min (-X), X-max (+X)
        (p000, p001, p011), (p000, p011, p010),
        (p100, p110, p111), (p100, p111, p101),
        # Y-min (-Y), Y-max (+Y)
        (p000, p100, p101), (p000, p101, p001),
        (p010, p011, p111), (p010, p111, p110),
    ]
    with open(str(path), "wb") as stream:
        stream.write(b"DfAM regression fixture: unit cube".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            # STL normal is intentionally zero; the tool computes its own.
            values = (0.0, 0.0, 0.0) + triangle[0] + triangle[1] + triangle[2]
            stream.write(struct.pack("<12fH", *values, 0))


class DfAMPreflightTests(unittest.TestCase):
    def test_binary_stl_cube_geometry_is_known(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "unit-cube.stl"
            write_unit_cube_stl(path)
            verts, tris = dfam.load_stl(str(path))
            stats = dfam.face_stats(verts, tris, dfam.DEFAULT_ANGLE)

        self.assertEqual(len(verts), 8)
        self.assertEqual(len(tris), 12)
        self.assertAlmostEqual(stats["volume"], 1.0, places=5)
        self.assertAlmostEqual(stats["area"], 6.0, places=5)
        self.assertEqual(stats["degenerate"], 0)

    def test_closed_cube_has_consistent_edges_and_positive_volume(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "unit-cube.stl"
            write_unit_cube_stl(path)
            verts, tris = dfam.load_stl(str(path))

        boundary, nonmanifold, inconsistent = dfam.edge_health(tris)
        self.assertEqual((boundary, nonmanifold, inconsistent), (0, 0, 0))
        self.assertGreater(dfam.face_stats(verts, tris, dfam.DEFAULT_ANGLE)["volume"], 0)

    def test_mechanics_are_never_reported_as_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "unit-cube.stl"
            write_unit_cube_stl(path)
            result = dfam.analyse(str(path), bed=(1.0, 1.0, 1.0))

        self.assertEqual(result["mechanics_status"]["state"], "UNKNOWN")
        self.assertTrue(all(
            check["state"] == "UNKNOWN"
            for check in result["mechanics_status"]["checks"]
        ))
        self.assertIn("not a mechanical validation", result["scope"])

    def test_cli_json_output_is_parseable_and_exits_successfully(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "unit-cube.stl"
            write_unit_cube_stl(path)
            completed = subprocess.run(
                [
                    sys.executable, str(SCRIPT), str(path),
                    "--json", "--bed", "1x1x1",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                check=False,
            )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["verdict"], "PASS")
        self.assertEqual(payload["verification"]["mechanical_validation"], "UNKNOWN")
        self.assertEqual(payload["metrics"]["box_mm"], [1.0, 1.0, 1.0])

    def test_cli_without_model_returns_usage_error(self):
        completed = subprocess.run(
            [sys.executable, str(SCRIPT)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("usage:", completed.stdout)

    def test_pose_pareto_filter_removes_dominated_candidate(self):
        rows = [
            {
                "support_ratio": 0.1,
                "height_mm": 10.0,
                "contact_mm2": 5.0,
                "flat_ceiling_mm2": 2.0,
            },
            {
                "support_ratio": 0.2,
                "height_mm": 12.0,
                "contact_mm2": 4.0,
                "flat_ceiling_mm2": 3.0,
            },
        ]
        flags, dominators = dfam.pareto_flags(rows)
        self.assertEqual(flags, [True, False])
        self.assertEqual(dominators, [0, 1])

    def test_cli_argument_parsers_accept_documented_forms(self):
        self.assertEqual(
            dfam.parse_bed("256x200x180"),
            (256.0, 200.0, 180.0),
        )
        self.assertEqual(
            dfam.parse_weights("support=.4,height=.2,contact=.2,ceiling=.2"),
            {
                "support_ratio": 0.4,
                "height_mm": 0.2,
                "contact_mm2": 0.2,
                "flat_ceiling_mm2": 0.2,
            },
        )
        with self.assertRaises(SystemExit):
            dfam.parse_weights("unknown=.5")


if __name__ == "__main__":
    unittest.main()
