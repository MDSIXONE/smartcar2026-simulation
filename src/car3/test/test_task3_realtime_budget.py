#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
WORLD = PACKAGE_DIR / "world" / "math.world"
URDF = PACKAGE_DIR / "urdf" / "car3.urdf"
PREPARE_LAUNCH = PACKAGE_DIR / "launch" / "task3_prepare.launch"
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
PLANNER_CONFIG = (
    WORKSPACE_DIR / "src" / "cym_planner" / "config" / "cym_planner_params.json"
)
PLANNER_SOURCE = WORKSPACE_DIR / "src" / "cym_planner" / "src" / "cym_planner.cpp"
GLOBAL_COSTMAP = (
    WORKSPACE_DIR
    / "src"
    / "gazebo_nav"
    / "launch"
    / "config"
    / "move_base"
    / "global_costmap_common.yaml"
)
LOCAL_COSTMAP = (
    WORKSPACE_DIR
    / "src"
    / "gazebo_nav"
    / "launch"
    / "config"
    / "move_base"
    / "local_costmap_common.yaml"
)


class Task3RealtimeBudgetTest(unittest.TestCase):
    def test_world_targets_one_to_one_simulation_time(self):
        root = ET.parse(WORLD).getroot()
        physics = root.find(".//world/physics")
        self.assertIsNotNone(physics)
        self.assertAlmostEqual(float(physics.findtext("max_step_size")), 0.01)
        self.assertAlmostEqual(
            float(physics.findtext("real_time_update_rate")), 100.0
        )
        self.assertAlmostEqual(float(physics.findtext("real_time_factor")), 1.0)
        self.assertEqual(root.findtext(".//world/scene/shadows"), "0")

    def test_unused_depth_rendering_is_lazy_and_rgb_is_ten_hertz(self):
        root = ET.parse(URDF).getroot()
        sensors = {
            sensor.get("name"): sensor
            for sensor in root.findall(".//gazebo/sensor")
        }
        rgb = sensors["rgb_camera"]
        depth = sensors["depth_camera"]
        self.assertEqual(rgb.findtext("update_rate"), "10")
        self.assertEqual(rgb.findtext("visualize"), "false")
        self.assertEqual(depth.findtext("always_on"), "false")
        self.assertEqual(depth.findtext("update_rate"), "5")
        self.assertEqual(
            depth.find("./plugin/alwaysOn").text, "false"
        )
        planar = root.find(".//gazebo/plugin[@name='planar_controller']")
        self.assertIsNotNone(planar)
        self.assertAlmostEqual(float(planar.findtext("cmdTimeout")), 0.10)

        planner = PLANNER_CONFIG.read_text(encoding="utf-8")
        self.assertIn('"safety_margin": 0.01', planner)
        self.assertIn('"max_vel_x": 0.6', planner)
        self.assertIn('"max_vel_theta": 2.0', planner)
        self.assertIn('"final_yaw_max_vel": 1.2', planner)
        planner_source = PLANNER_SOURCE.read_text(encoding="utf-8")
        self.assertIn("append_angular_candidate(0.0)", planner_source)
        self.assertIn(
            "append_angular_candidate(desired_angular_velocity * 0.25)",
            planner_source,
        )
        self.assertIn(
            "minimum_turn_velocity_, desired_angular_velocity", planner_source
        )
        global_footprint = next(
            line
            for line in GLOBAL_COSTMAP.read_text(encoding="utf-8").splitlines()
            if line.startswith("footprint:")
        )
        local_footprint = next(
            line
            for line in LOCAL_COSTMAP.read_text(encoding="utf-8").splitlines()
            if line.startswith("footprint:")
        )
        self.assertEqual(global_footprint, local_footprint)

    def test_fast_launch_and_task_have_wall_clock_guards(self):
        launch = ET.parse(PREPARE_LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default") for arg in launch.findall("arg")}
        self.assertEqual(args["gui"], "false")
        self.assertEqual(args["rviz"], "true")

        source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("task_wall_budget", source)
        self.assertIn("time.monotonic()", source)
        self.assertIn("position_only", source)
        self.assertIn("Odometry", source)
        self.assertIn("RTF preflight", source)


if __name__ == "__main__":
    unittest.main()
