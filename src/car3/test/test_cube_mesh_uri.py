#!/usr/bin/env python3
"""Regression test for portable Gazebo cube visual-mesh URIs."""

from pathlib import Path
import unittest
from xml.etree import ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CUBE_DIR = PACKAGE_DIR / "models" / "cube"
GAZEBO_LAUNCH = PACKAGE_DIR / "launch" / "v3_cym_gazebo.launch"


class CubeMeshUriTest(unittest.TestCase):
    def test_visual_meshes_use_the_portable_model_uri(self):
        for index in range(3):
            sdf_path = CUBE_DIR / f"model_{index}.sdf"
            root = ElementTree.parse(sdf_path).getroot()
            uri = root.findtext(".//visual/geometry/mesh/uri")

            self.assertEqual(uri, f"model://cube/meshes/cube_{index}.obj")
            self.assertTrue((CUBE_DIR / "meshes" / f"cube_{index}.obj").is_file())

    def test_gazebo_receives_project_model_path_before_starting(self):
        root = ElementTree.parse(GAZEBO_LAUNCH).getroot()
        children = list(root)
        model_path_index = next(
            index
            for index, child in enumerate(children)
            if child.tag == "env" and child.attrib.get("name") == "GAZEBO_MODEL_PATH"
        )
        gazebo_include_index = next(
            index
            for index, child in enumerate(children)
            if child.tag == "include" and "empty_world.launch" in child.attrib.get("file", "")
        )

        self.assertIn("$(find car3)/models", children[model_path_index].attrib["value"])
        self.assertLess(model_path_index, gazebo_include_index)


if __name__ == "__main__":
    unittest.main()
