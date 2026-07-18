#!/usr/bin/env python3
"""Regression test for portable Gazebo scene-label mesh URIs."""

from pathlib import Path
import unittest
from xml.etree import ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SIGN_DIR = PACKAGE_DIR / "models" / "sign"
WORLD_PATH = PACKAGE_DIR / "world" / "math.world"


class SignMeshUriTest(unittest.TestCase):
    def test_scene_labels_use_a_portable_gazebo_model(self):
        root = ElementTree.parse(WORLD_PATH).getroot()
        expected_meshes = {
            "wall_electronics": "wall_Electronics.obj",
            "wall_daily": "wall_Daily.obj",
            "wall_food": "wall_Food.obj",
        }

        self.assertTrue((SIGN_DIR / "model.config").is_file())
        self.assertTrue((SIGN_DIR / "model.sdf").is_file())

        for model_name, mesh_name in expected_meshes.items():
            model = root.find(f".//model[@name='{model_name}']")
            self.assertIsNotNone(model)
            uri = model.findtext(".//visual/geometry/mesh/uri")
            self.assertEqual(uri, f"model://sign/meshes/{mesh_name}")
            self.assertTrue((SIGN_DIR / "meshes" / mesh_name).is_file())


if __name__ == "__main__":
    unittest.main()
