"""Grid geometry, file format, and invalid configuration regressions."""

import copy
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from ros_fakes import SRC, load_module


maps = load_module("map_under_test", "nav/robot_navigation/scripts/generate_map.py")
PACKAGE = SRC / "nav/robot_navigation"


class MapTests(unittest.TestCase):
    def setUp(self):
        self.cfg = yaml.safe_load((PACKAGE / "config/field_config.yaml").read_text())

    def test_default_grid_geometry_and_cell_types(self):
        grid, info = maps.build_grid(self.cfg)
        self.assertEqual(grid.shape, (122, 173))
        self.assertEqual(info["offset_cells"], 11)
        self.assertEqual(info["resolution_m"], 0.02)
        self.assertEqual(np.count_nonzero(grid == maps.FREE), 151 * 100)
        self.assertTrue(np.all(grid[11:111, 11:162] == maps.FREE))
        self.assertTrue(np.all(grid[10, 10:163] == maps.OCCUPIED))
        self.assertTrue(np.all(grid[:10] == maps.UNKNOWN))

    def test_zero_margin_retains_wall_ring(self):
        self.cfg["map"]["margin_mm"] = 0
        grid, info = maps.build_grid(self.cfg)
        self.assertTrue(np.all(grid[0] == maps.OCCUPIED))
        self.assertTrue(np.all(grid[-1] == maps.OCCUPIED))
        self.assertFalse(np.any(grid == maps.UNKNOWN))
        self.assertEqual(info["offset_cells"], 1)

    def test_pgm_flip_and_metadata_origin(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            grid = np.array([[0, 254], [205, 0]], dtype=np.uint8)
            maps.write_pgm(path / "map.pgm", grid)
            self.assertEqual((path / "map.pgm").read_bytes(),
                             b"P5\n2 2\n255\n" + bytes([205, 0, 0, 254]))
            _, info = maps.build_grid(self.cfg)
            maps.write_map_yaml(path / "map.yaml", "map.pgm", info)
            meta = yaml.safe_load((path / "map.yaml").read_text())
            self.assertEqual(meta["origin"], [-0.22, -0.22, 0.0])
            self.assertEqual(meta["image"], "map.pgm")

    def test_default_generation_matches_checked_in_map(self):
        grid, info = maps.build_grid(self.cfg)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.pgm"
            maps.write_pgm(path, grid)
            self.assertEqual(path.read_bytes(), (PACKAGE / "maps/field_map.pgm").read_bytes())

    def test_invalid_geometry_rejected(self):
        cases = [("map", "resolution_mm", 0), ("map", "resolution_mm", math.nan),
                 ("map", "resolution_mm", 10000), ("map", "resolution_mm", 0.00001),
                 ("map", "margin_mm", -20), ("field", "length_mm", -1),
                 ("field", "width_mm", math.inf), ("field", "wall_thickness_mm", 0)]
        for section, key, value in cases:
            cfg = copy.deepcopy(self.cfg)
            cfg[section][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                maps.build_grid(cfg)

    def test_cli_writes_files_to_requested_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            process = subprocess.run(
                [sys.executable, str(PACKAGE / "scripts/generate_map.py"),
                 "--out-dir", folder], capture_output=True, text=True, check=False)
            self.assertEqual(process.returncode, 0, process.stderr)
            self.assertTrue((Path(folder) / "field_map.pgm").is_file())
            self.assertTrue((Path(folder) / "field_map.yaml").is_file())

    def test_cli_rejects_name_escaping_output_directory(self):
        self.cfg["map"]["name"] = "../escaped"
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            config = path / "config.yaml"
            config.write_text(yaml.safe_dump(self.cfg))
            process = subprocess.run(
                [sys.executable, str(PACKAGE / "scripts/generate_map.py"),
                 "--config", str(config), "--out-dir", str(path / "output")],
                capture_output=True, text=True, check=False)
            self.assertEqual(process.returncode, 2)
            self.assertIn("map.name", process.stderr)
            self.assertFalse((path / "escaped.pgm").exists())


if __name__ == "__main__":
    unittest.main()
