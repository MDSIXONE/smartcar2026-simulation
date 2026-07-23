#!/usr/bin/env python3
"""Regression tests for camera-only task-three pickup selection."""

import importlib.util
import json
from pathlib import Path
import unittest

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
VISION_CONFIG = PACKAGE_DIR / "config" / "task3_vision.yaml"
VISION_DIR = PACKAGE_DIR / "models" / "vision"
DATASET_DIR = WORKSPACE_DIR / "datasets" / "cube_yolov5"


class Task3VisualPipelineTest(unittest.TestCase):
    def test_runtime_source_does_not_read_gazebo_cube_positions(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("/gazebo/get_model_state", source)
        self.assertNotIn("GetModelState", source)
        self.assertNotIn("_cube_poses", source)
        self.assertNotIn("cube_world_poses", source)
        self.assertNotIn("time.time()", source)
        self.assertIn("rospy.Time.now()", source)

    def test_search_order_and_grasp_calibration_are_complete(self):
        config = yaml.safe_load(VISION_CONFIG.read_text(encoding="utf-8"))
        self.assertGreaterEqual(config["vision_scan_center_tolerance"], 0.05)
        self.assertGreaterEqual(config["vision_scan_vote_frames"], 5)
        regions = config["vision_search_regions"]
        self.assertEqual(
            [region["name"] for region in regions],
            ["left", "middle", "right"],
        )
        for region in regions:
            self.assertEqual(len(region["observation_goal"]), 3)
            self.assertEqual(len(region["recorded_bbox_px"]), 4)
            self.assertEqual(len(region["grasp_target"]), 4)
            self.assertEqual(
                set(region["grasp_acceptance"]),
                {"center_x", "center_y", "width", "height"},
            )

    def test_bottom_center_samples_are_recognised_and_graspable(self):
        try:
            import cv2
            import onnxruntime as ort
        except ImportError as error:
            self.skipTest("vision runtime is not installed: %s" % error)

        spec = importlib.util.spec_from_file_location("task3_pick_deliver", TASK_SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        config = yaml.safe_load(VISION_CONFIG.read_text(encoding="utf-8"))
        regions = {
            region["name"]: region
            for region in config["vision_search_regions"]
        }

        detector = module.PickDeliverTask.__new__(module.PickDeliverTask)
        detector.vision_class_names = config["vision_class_names"]
        detector.target_class_id = 0
        detector.vision_confidence = config["vision_confidence_threshold"]
        detector.vision_nms = config["vision_nms_threshold"]
        detector.vision_input_size = config["vision_input_size"]
        detector.vision_session = ort.InferenceSession(
            str(VISION_DIR / "cube_yolov5_best.onnx"),
            providers=["CPUExecutionProvider"],
        )
        detector.vision_input_name = detector.vision_session.get_inputs()[0].name
        detector.vision_output_name = detector.vision_session.get_outputs()[0].name
        detector.vision_hog = cv2.HOGDescriptor(
            (64, 64), (16, 16), (8, 8), (8, 8), 9
        )
        detector.vision_label_guard = cv2.ml.SVM_load(
            str(VISION_DIR / "cube_label_hog_svm.xml")
        )
        detector._publish_vision_debug = lambda *_args: None

        checked = set()
        checked_combinations = set()
        for line in (DATASET_DIR / "metadata.jsonl").read_text(
            encoding="utf-8"
        ).splitlines():
            record = json.loads(line)
            if record["grid_cell"] != "bottom_center":
                continue
            region_name = (
                "middle" if record["target_region"] == "upper"
                else record["target_region"]
            )
            image = cv2.imread(str(DATASET_DIR / record["image"]))
            detections = detector._detect(image, regions[region_name])
            self.assertTrue(detections, record["image"])
            self.assertEqual(detections[0]["class_name"], record["class_name"])
            self.assertTrue(
                detector._inside_grasp_range(detections[0], regions[region_name]),
                "%s -> %s" % (record["image"], detections[0]),
            )
            checked.add(record["class_name"])
            checked_combinations.add(
                (record["class_name"], record["target_region"])
            )
        self.assertEqual(checked, {"food", "daily", "electronics"})
        self.assertEqual(
            checked_combinations,
            {
                (class_name, region_name)
                for class_name in ("food", "daily", "electronics")
                for region_name in ("left", "upper", "right")
            },
        )


if __name__ == "__main__":
    unittest.main()
