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
    @staticmethod
    def _make_detector(config, cv2, ort):
        spec = importlib.util.spec_from_file_location(
            "task3_pick_deliver", TASK_SCRIPT
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
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
        detector.vision_template_min_score = config["vision_template_min_score"]
        detector.vision_template_min_margin = config["vision_template_min_margin"]
        detector.vision_label_templates = []
        template_dir = PACKAGE_DIR / "models" / "cube" / "meshes"
        for filename in (
            "Food.png", "Daily_Necessities.png", "Electronics.png"
        ):
            template = cv2.imread(
                str(template_dir / filename), cv2.IMREAD_GRAYSCALE
            )
            detector.vision_label_templates.append(
                cv2.threshold(
                    template,
                    0,
                    255,
                    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
                )[1]
            )
        detector._publish_vision_debug = lambda *_args: None
        return detector

    def test_runtime_source_does_not_read_gazebo_cube_positions(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("/gazebo/get_model_state", source)
        self.assertNotIn("GetModelState", source)
        self.assertNotIn("_cube_poses", source)
        self.assertNotIn("cube_world_poses", source)
        self.assertNotIn("time.time()", source)
        self.assertIn("rospy.Time.now()", source)
        search_body = source[
            source.index("def _find_and_align_target"):
            source.index("def _move_base", source.index("def _find_and_align_target"))
        ]
        self.assertLess(
            search_body.index("_quick_classify_observation"),
            search_body.index("_vision_align"),
        )
        self.assertLess(
            search_body.index("_vision_align"),
            search_body.index("_classify_aligned_cube"),
        )
        self.assertNotIn("_classify_cube_multiview", source)
        self.assertNotIn("_collect_classification_view", source)
        self.assertNotIn("vision_label_guard", source)

    def test_search_order_and_grasp_calibration_are_complete(self):
        config = yaml.safe_load(VISION_CONFIG.read_text(encoding="utf-8"))
        self.assertGreaterEqual(config["vision_scan_center_tolerance"], 0.05)
        self.assertGreaterEqual(config["vision_quick_classify_frames"], 3)
        self.assertGreaterEqual(config["vision_quick_min_confidence"], 0.85)
        self.assertGreater(config["vision_quick_classify_timeout"], 0.0)
        self.assertGreaterEqual(config["vision_classify_stable_frames"], 5)
        self.assertGreater(config["vision_classify_timeout"], 0.0)
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

        config = yaml.safe_load(VISION_CONFIG.read_text(encoding="utf-8"))
        regions = {
            region["name"]: region
            for region in config["vision_search_regions"]
        }

        detector = self._make_detector(config, cv2, ort)

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

    def test_new_yolo_supports_safe_observation_skip(self):
        try:
            import cv2
            import onnxruntime as ort
        except ImportError as error:
            self.skipTest("vision runtime is not installed: %s" % error)

        config = yaml.safe_load(VISION_CONFIG.read_text(encoding="utf-8"))
        regions = {
            region["name"]: region
            for region in config["vision_search_regions"]
        }
        detector = self._make_detector(config, cv2, ort)
        eligible_for_early_decision = 0
        centered_eligible = 0
        centered_total = 0
        total = 0
        for line in (DATASET_DIR / "metadata.jsonl").read_text(
            encoding="utf-8"
        ).splitlines():
            record = json.loads(line)
            region_name = (
                "middle" if record["target_region"] == "upper"
                else record["target_region"]
            )
            image = cv2.imread(str(DATASET_DIR / record["image"]))
            detections = detector._detect(image, regions[region_name])
            self.assertTrue(detections, record["image"])
            detection = detections[0]
            self.assertEqual(
                detection["yolo_class_name"],
                record["class_name"],
                record["image"],
            )
            self.assertGreaterEqual(
                detection["confidence"], 0.70, record["image"]
            )
            if detection["confidence"] >= config["vision_quick_min_confidence"]:
                eligible_for_early_decision += 1
                if record["grid_column"] == 1:
                    centered_eligible += 1
            if record["grid_column"] == 1:
                centered_total += 1
            total += 1
        self.assertGreaterEqual(eligible_for_early_decision, 40)
        self.assertGreaterEqual(centered_eligible, 44)
        self.assertEqual(centered_total, 45)
        self.assertEqual(total, 135)


if __name__ == "__main__":
    unittest.main()
