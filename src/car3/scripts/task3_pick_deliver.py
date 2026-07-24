#!/usr/bin/env python3
"""Find, visually align with, pick, and deliver one labelled cube.

The task visits fixed left/middle/right observation poses and runs the trained
YOLOv5 model on the RGB camera.  The requested class and its image-space box
are the only inputs used to choose a bay and align the base for grasping.
Gazebo cube positions are intentionally never read by this runtime node.
"""

import math
import os
import threading

import actionlib
import cv2
import numpy as np
import rospy
import tf.transformations as transformations
from actionlib_msgs.msg import GoalStatus
from controller_manager_msgs.srv import (
    ListControllers,
    SwitchController,
    SwitchControllerRequest,
)
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetLinkState, SetModelState
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
# Model names are retained only for the post-close Gazebo attachment fallback.
# They are never queried to locate a cube or choose a pickup bay.
CATEGORY_CUBE = {"food": "cube_0", "daily": "cube_1", "electronics": "cube_2"}
CATEGORY_ALIASES = {
    "food": "food", "foods": "food", "食品": "food", "食品类": "food",
    "daily": "daily", "daily_necessities": "daily", "日用品": "daily",
    "electronics": "electronics", "electronic": "electronics", "电子": "electronics",
    "电子产品": "electronics",
}
ITEM_CATEGORIES = {
    "可乐": "food", "牛奶": "food", "面包": "food", "饼干": "food", "苹果": "food",
    "香蕉": "food", "零食": "food", "饮料": "food",
    "牙刷": "daily", "毛巾": "daily", "纸巾": "daily", "肥皂": "daily",
    "洗发水": "daily", "水杯": "daily",
    "手机": "electronics", "平板": "electronics", "耳机": "electronics",
    "键盘": "electronics", "鼠标": "electronics", "相机": "electronics",
    "充电器": "electronics",
}
WAREHOUSES = {
    "food": ("食品加工车间", (1.00, -2.98, -math.pi / 2.0)),
    "daily": ("日用品加工车间", (1.00, -1.50, math.pi / 2.0)),
    "electronics": ("电子产品生产车间", (2.55, -2.22, 0.0)),
}

def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def quaternion_from_yaw(yaw):
    x, y, z, w = transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


class PickDeliverTask:
    def __init__(self):
        rospy.init_node("task3_pick_deliver")

        self.cargo_item = str(rospy.get_param("~cargo_item", "")).strip()
        requested_category = str(rospy.get_param("~cargo_category", "auto")).strip().lower()
        self.category = self._resolve_category(requested_category, self.cargo_item)
        self.cargo_name = str(rospy.get_param("~cargo_name", "")).strip()
        if not self.cargo_name:
            self.cargo_name = self.cargo_item or self.category
        self.cargo_model = CATEGORY_CUBE[self.category]
        self.destination_name, self.destination = WAREHOUSES[self.category]

        self.arm_grasp = self._pose_param(
            "~arm_grasp_pose", [-0.0001, 1.5000, 0.2800, 1.3000, 0.0000]
        )
        self.arm_carry = self._pose_param(
            "~arm_carry_pose", [-0.0001, 0.0000, -1.7200, -0.5000, 0.0000]
        )
        self.arm_grasp_duration = float(rospy.get_param("~arm_grasp_duration", 2.0))
        self.arm_carry_duration = float(rospy.get_param("~arm_carry_duration", 2.5))
        self.nav_timeout = float(rospy.get_param("~nav_timeout", 110.0))
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        self.vision_model_path = os.path.abspath(
            os.path.expanduser(str(rospy.get_param("~vision_model_path")))
        )
        self.vision_label_template_dir = os.path.abspath(
            os.path.expanduser(str(rospy.get_param("~vision_label_template_dir")))
        )
        self.vision_class_names = [
            str(name) for name in rospy.get_param(
                "~vision_class_names", ["food", "daily", "electronics"]
            )
        ]
        if self.category not in self.vision_class_names:
            raise rospy.ROSException(
                "Requested category %s is absent from vision_class_names=%s"
                % (self.category, self.vision_class_names)
            )
        self.target_class_id = self.vision_class_names.index(self.category)
        self.vision_confidence = float(
            rospy.get_param("~vision_confidence_threshold", 0.20)
        )
        self.vision_nms = float(rospy.get_param("~vision_nms_threshold", 0.45))
        self.vision_input_size = int(rospy.get_param("~vision_input_size", 640))
        self.vision_scan_timeout = float(rospy.get_param("~vision_scan_timeout", 8.0))
        self.vision_scan_center_tolerance = float(
            rospy.get_param("~vision_scan_center_tolerance", 0.060)
        )
        self.vision_quick_classify_frames = int(
            rospy.get_param("~vision_quick_classify_frames", 5)
        )
        self.vision_quick_classify_timeout = float(
            rospy.get_param("~vision_quick_classify_timeout", 1.5)
        )
        self.vision_quick_min_confidence = float(
            rospy.get_param("~vision_quick_min_confidence", 0.75)
        )
        self.vision_classify_stable_frames = int(
            rospy.get_param("~vision_classify_stable_frames", 7)
        )
        self.vision_classify_timeout = float(
            rospy.get_param("~vision_classify_timeout", 5.0)
        )
        self.vision_template_min_score = float(
            rospy.get_param("~vision_template_min_score", 0.30)
        )
        self.vision_template_min_margin = float(
            rospy.get_param("~vision_template_min_margin", 0.08)
        )
        self.vision_align_timeout = float(rospy.get_param("~vision_align_timeout", 25.0))
        self.vision_lost_timeout = float(rospy.get_param("~vision_lost_timeout", 2.5))
        self.vision_align_stable_frames = int(
            rospy.get_param("~vision_align_stable_frames", 5)
        )
        self.vision_forward_gain = float(rospy.get_param("~vision_forward_gain", 0.45))
        self.vision_angular_gain = float(rospy.get_param("~vision_angular_gain", 1.80))
        self.vision_max_forward = float(
            rospy.get_param("~vision_max_forward_speed", 0.08)
        )
        self.vision_max_angular = float(
            rospy.get_param("~vision_max_angular_speed", 0.40)
        )
        self.search_regions = self._read_search_regions(
            rospy.get_param("~vision_search_regions")
        )

        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", Float64, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.vision_debug_pub = rospy.Publisher(
            "/sim_task3/vision/debug_image", Image, queue_size=1
        )
        self.status_pub = rospy.Publisher("/sim_task3/status", String, queue_size=10, latch=True)
        self.done_pub = rospy.Publisher("/sim_task3/done", Bool, queue_size=1, latch=True)
        self.carry_mode_pub = rospy.Publisher(
            "/sim_task3/carry_mode", Bool, queue_size=1, latch=True
        )
        # task3_prepare keeps the parked pose through Gazebo configuration,
        # not through arm control.  This topic asks that holder to stop before
        # arm_controller takes ownership at the pickup bay.
        self.arm_control_enabled_pub = rospy.Publisher(
            "/sim_task3/arm_control_enabled", Bool, queue_size=1, latch=False
        )

        self.image_lock = threading.Lock()
        self.latest_image = None
        self.latest_image_sequence = 0
        self.grasp_state = "UNKNOWN"
        self.fallback_holding = False
        self.fallback_hold_timer = None
        rospy.Subscriber(self.camera_topic, Image, self._camera_callback, queue_size=1)
        rospy.Subscriber("/grasp_attach/state", String, self._grasp_state_callback, queue_size=1)

        self.nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.switch_controllers = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )
        self.list_controllers = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers
        )
        self.get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self._load_vision_model()
        # A fresh task must never inherit low-speed mode from an earlier run.
        self.carry_mode_pub.publish(Bool(data=False))

    @staticmethod
    def _resolve_category(requested, item):
        if requested and requested != "auto":
            category = CATEGORY_ALIASES.get(requested)
            if category is None:
                raise rospy.ROSException(
                    "Unknown cargo_category '%s'; use food/食品, daily/日用品, or electronics/电子产品"
                    % requested
                )
            return category
        candidate = str(item).strip().lower()
        category = CATEGORY_ALIASES.get(candidate) or ITEM_CATEGORIES.get(candidate)
        if category is None:
            raise rospy.ROSException(
                "Cannot infer category for cargo_item '%s'; pass cargo_category explicitly" % item
            )
        return category

    @staticmethod
    def _pose_param(name, default):
        pose = rospy.get_param(name, default)
        if isinstance(pose, str):
            pose = [value.strip() for value in pose.split(",") if value.strip()]
        if not isinstance(pose, (list, tuple)) or len(pose) != len(ARM_JOINTS):
            raise rospy.ROSException("%s must contain five joint angles" % name)
        return [float(value) for value in pose]

    @staticmethod
    def _read_search_regions(regions):
        if not isinstance(regions, list) or len(regions) != 3:
            raise rospy.ROSException(
                "vision_search_regions must contain left, middle, and right entries"
            )
        parsed = []
        for region in regions:
            if not isinstance(region, dict):
                raise rospy.ROSException("each vision search region must be a mapping")
            missing = [
                key for key in (
                    "name", "display_name", "observation_goal",
                    "grasp_target", "grasp_acceptance",
                )
                if key not in region
            ]
            if missing:
                raise rospy.ROSException(
                    "vision region is missing keys: %s" % ", ".join(missing)
                )
            goal = [float(value) for value in region["observation_goal"]]
            target = [float(value) for value in region["grasp_target"]]
            if len(goal) != 3 or len(target) != 4:
                raise rospy.ROSException(
                    "%s observation_goal/grasp_target dimensions are invalid"
                    % region["name"]
                )
            acceptance = {}
            for key in ("center_x", "center_y", "width", "height"):
                values = [
                    float(value) for value in region["grasp_acceptance"].get(key, [])
                ]
                if len(values) != 2 or values[0] > values[1]:
                    raise rospy.ROSException(
                        "%s grasp_acceptance.%s must be [minimum, maximum]"
                        % (region["name"], key)
                    )
                acceptance[key] = values
            parsed.append({
                "name": str(region["name"]),
                "display_name": str(region["display_name"]),
                "observation_goal": goal,
                "grasp_target": target,
                "grasp_acceptance": acceptance,
            })
        if [region["name"] for region in parsed] != ["left", "middle", "right"]:
            raise rospy.ROSException(
                "vision_search_regions order must be exactly left, middle, right"
            )
        return parsed

    def _load_vision_model(self):
        if not os.path.isfile(self.vision_model_path):
            raise rospy.ROSException(
                "YOLOv5 ONNX model does not exist: %s" % self.vision_model_path
            )
        if not os.path.isdir(self.vision_label_template_dir):
            raise rospy.ROSException(
                "visual label template directory does not exist: %s"
                % self.vision_label_template_dir
            )
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise rospy.ROSException(
                "onnxruntime is required; from the workspace root run "
                "'python3 -m pip install -r requirements-vision.txt'"
            ) from error

        options = ort.SessionOptions()
        options.intra_op_num_threads = max(1, int(rospy.get_param("~vision_cpu_threads", 2)))
        options.inter_op_num_threads = 1
        self.vision_session = ort.InferenceSession(
            self.vision_model_path,
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        inputs = self.vision_session.get_inputs()
        outputs = self.vision_session.get_outputs()
        if len(inputs) != 1 or not outputs:
            raise rospy.ROSException("unexpected YOLOv5 ONNX input/output signature")
        self.vision_input_name = inputs[0].name
        self.vision_output_name = outputs[0].name
        template_files = {
            "food": "Food.png",
            "daily": "Daily_Necessities.png",
            "electronics": "Electronics.png",
        }
        self.vision_label_templates = []
        for class_name in self.vision_class_names:
            if class_name not in template_files:
                raise rospy.ROSException(
                    "no visual label template configured for class %s" % class_name
                )
            path = os.path.join(
                self.vision_label_template_dir, template_files[class_name]
            )
            template = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if template is None:
                raise rospy.ROSException(
                    "cannot read visual label template: %s" % path
                )
            template = cv2.resize(
                template, (128, 128), interpolation=cv2.INTER_AREA
            )
            self.vision_label_templates.append(
                cv2.threshold(
                    template,
                    0,
                    255,
                    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
                )[1]
            )
        warmup = np.zeros(
            (1, 3, self.vision_input_size, self.vision_input_size), dtype=np.float32
        )
        output = self.vision_session.run(
            [self.vision_output_name], {self.vision_input_name: warmup}
        )[0]
        if output.ndim != 3 or output.shape[-1] != 5 + len(self.vision_class_names):
            raise rospy.ROSException(
                "YOLOv5 output shape %s does not match %d configured classes"
                % (output.shape, len(self.vision_class_names))
            )
        self._status(
            "YOLOv5 vision ready: %s; classes=%s"
            % (os.path.basename(self.vision_model_path), ",".join(self.vision_class_names))
        )

    @staticmethod
    def _image_message_to_bgr(message):
        encoding = str(message.encoding).lower()
        channels = {
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
            "mono8": 1,
        }.get(encoding)
        if channels is None:
            raise ValueError("unsupported camera encoding %s" % message.encoding)
        row_bytes = int(message.width) * channels
        if int(message.step) < row_bytes:
            raise ValueError("camera image step is shorter than one pixel row")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        needed = int(message.height) * int(message.step)
        if raw.size < needed:
            raise ValueError("camera image data is truncated")
        rows = raw[:needed].reshape((int(message.height), int(message.step)))
        pixels = rows[:, :row_bytes].reshape(
            (int(message.height), int(message.width), channels)
        )
        if encoding == "bgr8":
            return pixels.copy()
        if encoding == "rgb8":
            return pixels[:, :, ::-1].copy()
        if encoding == "bgra8":
            return pixels[:, :, :3].copy()
        if encoding == "rgba8":
            return pixels[:, :, [2, 1, 0]].copy()
        return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)

    def _camera_callback(self, message):
        try:
            image = self._image_message_to_bgr(message)
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Camera frame rejected: %s" % error)
            return
        with self.image_lock:
            self.latest_image = image
            self.latest_image_sequence += 1

    def _grasp_state_callback(self, message):
        self.grasp_state = message.data

    def _status(self, text):
        rospy.loginfo(text)
        self.status_pub.publish(String(data=text))

    def _wait_for_services(self):
        for name in (
            "/gazebo/get_link_state",
            "/gazebo/set_model_state",
            "/controller_manager/switch_controller",
            "/controller_manager/list_controllers",
        ):
            self._status("Waiting for %s" % name)
            rospy.wait_for_service(name)
        self._status("Waiting for move_base")
        self.nav.wait_for_server()

    def _latest_frame(self, after_sequence):
        with self.image_lock:
            if (
                self.latest_image is None
                or self.latest_image_sequence == after_sequence
            ):
                return None
            return self.latest_image_sequence, self.latest_image.copy()

    def _letterbox(self, image):
        height, width = image.shape[:2]
        scale = min(
            float(self.vision_input_size) / float(width),
            float(self.vision_input_size) / float(height),
        )
        resized_width = int(round(width * scale))
        resized_height = int(round(height * scale))
        resized = cv2.resize(
            image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
        )
        pad_x = (self.vision_input_size - resized_width) // 2
        pad_y = (self.vision_input_size - resized_height) // 2
        canvas = np.full(
            (self.vision_input_size, self.vision_input_size, 3), 114, dtype=np.uint8
        )
        canvas[
            pad_y:pad_y + resized_height, pad_x:pad_x + resized_width
        ] = resized
        rgb = canvas[:, :, ::-1].transpose((2, 0, 1))
        blob = np.ascontiguousarray(rgb, dtype=np.float32) / 255.0
        return blob[np.newaxis, :], scale, pad_x, pad_y

    @staticmethod
    def _ordered_quad(points):
        points = points.reshape(-1, 2).astype(np.float32)
        sums = points.sum(axis=1)
        differences = np.diff(points, axis=1).reshape(-1)
        return np.asarray(
            [
                points[np.argmin(sums)],
                points[np.argmin(differences)],
                points[np.argmax(sums)],
                points[np.argmax(differences)],
            ],
            dtype=np.float32,
        )

    def _template_classify(self, image, box):
        """Rectify the bright printed face and compare it with known labels."""
        x1, y1, x2, y2 = [float(value) for value in box]
        width = x2 - x1
        height = y2 - y1
        x1 = max(0, int(round(x1 - 0.20 * width)))
        y1 = max(0, int(round(y1 - 0.20 * height)))
        x2 = min(image.shape[1], int(round(x2 + 0.20 * width)))
        y2 = min(image.shape[0], int(round(y2 + 0.20 * height)))
        crop = image[y1:y2, x1:x2]
        if crop.size == 0:
            return None, 0.0, 0.0
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(
            cv2.GaussianBlur(gray, (3, 3), 0),
            205,
            255,
            cv2.THRESH_BINARY,
        )[1]
        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[0]
        candidates = []
        minimum_area = 0.04 * crop.shape[0] * crop.shape[1]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < minimum_area:
                continue
            polygon = cv2.approxPolyDP(
                contour, 0.035 * cv2.arcLength(contour, True), True
            )
            if len(polygon) == 4:
                candidates.append((area, polygon))
        if not candidates:
            return None, 0.0, 0.0

        _, polygon = max(candidates, key=lambda item: item[0])
        destination = np.asarray(
            [[0, 0], [127, 0], [127, 127], [0, 127]],
            dtype=np.float32,
        )
        rectified = cv2.warpPerspective(
            gray,
            cv2.getPerspectiveTransform(
                self._ordered_quad(polygon), destination
            ),
            (128, 128),
        )
        binary = cv2.threshold(
            rectified,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
        )[1]
        scores = [
            float(
                np.corrcoef(
                    binary.reshape(-1), template.reshape(-1)
                )[0, 1]
            )
            for template in self.vision_label_templates
        ]
        class_id = int(np.argmax(scores))
        ranked = sorted(scores, reverse=True)
        score = ranked[0]
        margin = ranked[0] - ranked[1]
        if (
            not math.isfinite(score)
            or score < self.vision_template_min_score
            or margin < self.vision_template_min_margin
        ):
            return None, score, margin
        return class_id, score, margin

    def _detect(self, image, region):
        blob, scale, pad_x, pad_y = self._letterbox(image)
        output = self.vision_session.run(
            [self.vision_output_name], {self.vision_input_name: blob}
        )[0]
        rows = np.squeeze(output, axis=0)
        if rows.shape[0] <= 5 + len(self.vision_class_names):
            rows = rows.transpose()

        image_height, image_width = image.shape[:2]
        candidates = []
        for row in rows:
            objectness = float(row[4])
            if objectness < self.vision_confidence:
                continue
            class_scores = row[5:5 + len(self.vision_class_names)]
            class_id = int(np.argmax(class_scores))
            confidence = objectness * float(class_scores[class_id])
            if confidence < self.vision_confidence:
                continue

            center_x, center_y, box_width, box_height = [
                float(value) for value in row[:4]
            ]
            x1 = clamp((center_x - box_width / 2.0 - pad_x) / scale, 0.0, image_width - 1.0)
            y1 = clamp((center_y - box_height / 2.0 - pad_y) / scale, 0.0, image_height - 1.0)
            x2 = clamp((center_x + box_width / 2.0 - pad_x) / scale, 0.0, image_width - 1.0)
            y2 = clamp((center_y + box_height / 2.0 - pad_y) / scale, 0.0, image_height - 1.0)
            if x2 <= x1 or y2 <= y1:
                continue
            candidates.append({
                "yolo_class_id": class_id,
                "yolo_class_name": self.vision_class_names[class_id],
                "confidence": confidence,
                "box_px": [x1, y1, x2, y2],
                "nms_box": [
                    int(round(x1)), int(round(y1)),
                    int(round(x2 - x1)), int(round(y2 - y1)),
                ],
            })

        detections = []
        indices = cv2.dnn.NMSBoxes(
            [item["nms_box"] for item in candidates],
            [item["confidence"] for item in candidates],
            self.vision_confidence,
            self.vision_nms,
        ) if candidates else []
        for index in np.asarray(indices).reshape(-1):
            item = candidates[int(index)]
            class_id, template_score, template_margin = self._template_classify(
                image, item["box_px"]
            )
            item["template_class_id"] = class_id
            item["template_score"] = template_score
            item["template_margin"] = template_margin
            item["template_class_name"] = (
                self.vision_class_names[class_id]
                if class_id is not None
                else "uncertain"
            )
            if class_id is None:
                class_id = item["yolo_class_id"]
            item["class_id"] = class_id
            item["class_name"] = self.vision_class_names[class_id]
            x1, y1, x2, y2 = item["box_px"]
            item["center_x"] = ((x1 + x2) / 2.0) / float(image_width)
            item["center_y"] = ((y1 + y2) / 2.0) / float(image_height)
            item["width"] = (x2 - x1) / float(image_width)
            item["height"] = (y2 - y1) / float(image_height)
            detections.append(item)
        detections.sort(key=lambda item: item["confidence"], reverse=True)
        self._publish_vision_debug(image, detections, region)
        return detections

    def _publish_vision_debug(self, image, detections, region):
        annotated = image.copy()
        image_height, image_width = annotated.shape[:2]
        target = region["grasp_target"]
        target_x = int(round(target[0] * image_width))
        target_y = int(round(target[1] * image_height))
        target_w = int(round(target[2] * image_width))
        target_h = int(round(target[3] * image_height))
        cv2.rectangle(
            annotated,
            (target_x - target_w // 2, target_y - target_h // 2),
            (target_x + target_w // 2, target_y + target_h // 2),
            (255, 255, 0),
            2,
        )
        cv2.drawMarker(
            annotated,
            (target_x, target_y),
            (255, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )
        for detection in detections:
            x1, y1, x2, y2 = [int(round(value)) for value in detection["box_px"]]
            colour = (
                (0, 220, 0)
                if detection["class_id"] == self.target_class_id
                else (0, 150, 255)
            )
            cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(
                annotated,
                "%s (template:%.2f yolo:%s %.2f)"
                % (
                    detection["template_class_name"],
                    detection["template_score"],
                    detection["yolo_class_name"],
                    detection["confidence"],
                ),
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                colour,
                2,
                cv2.LINE_AA,
            )
        message = Image()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "camera_rgb_optical_frame"
        message.height = image_height
        message.width = image_width
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = image_width * 3
        message.data = annotated.tobytes()
        self.vision_debug_pub.publish(message)

    def _scan_region(self, region):
        deadline = rospy.Time.now() + rospy.Duration(self.vision_scan_timeout)
        last_sequence = -1
        seen_frames = 0
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    rate.sleep()
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if not detections:
                    self.cmd_pub.publish(Twist())
                    rate.sleep()
                    continue

                cube = detections[0]
                seen_frames += 1
                horizontal_error = region["grasp_target"][0] - cube["center_x"]
                if abs(horizontal_error) > self.vision_scan_center_tolerance:
                    command = Twist()
                    command.angular.z = clamp(
                        self.vision_angular_gain * horizontal_error,
                        -0.25,
                        0.25,
                    )
                    command.angular.z = self._ensure_minimum_speed(
                        command.angular.z, 0.055
                    )
                    self.cmd_pub.publish(command)
                    rate.sleep()
                    continue

                self.cmd_pub.publish(Twist())
                self._status(
                    "Camera acquired a cube in %s region "
                    "(YOLO raw=%s, confidence=%.3f); starting observation classification"
                    % (
                        region["display_name"],
                        cube["yolo_class_name"],
                        cube["confidence"],
                    )
                )
                return cube
        finally:
            self.cmd_pub.publish(Twist())
        self._status(
            "No cube acquired in %s region; detector frames=%d"
            % (region["display_name"], seen_frames)
        )
        return None

    def _quick_classify_observation(self, region, initial_detection):
        """Return a class only when YOLO and the printed face agree repeatedly.

        This fast path is allowed to skip a confidently non-target region while
        the vehicle is still at its observation pose.  Any disagreement,
        low-confidence frame, or timeout returns None so the existing
        close-range alignment and seven-frame verification remains the safe
        fallback.
        """
        deadline = rospy.Time.now() + rospy.Duration(
            self.vision_quick_classify_timeout
        )
        last_sequence = -1
        tracked_center = (
            initial_detection["center_x"],
            initial_detection["center_y"],
        )
        votes = []
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    rate.sleep()
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if not detections:
                    votes = []
                    rate.sleep()
                    continue
                cube = min(
                    detections,
                    key=lambda item: (
                        (item["center_x"] - tracked_center[0]) ** 2
                        + (item["center_y"] - tracked_center[1]) ** 2
                    ),
                )
                tracking_distance = math.hypot(
                    cube["center_x"] - tracked_center[0],
                    cube["center_y"] - tracked_center[1],
                )
                tracked_center = (cube["center_x"], cube["center_y"])
                yolo_id = cube["yolo_class_id"]
                template_id = cube["template_class_id"]
                reliable = (
                    tracking_distance <= 0.25
                    and abs(
                        region["grasp_target"][0] - cube["center_x"]
                    ) <= self.vision_scan_center_tolerance
                    and cube["confidence"] >= self.vision_quick_min_confidence
                    and template_id is not None
                    and template_id == yolo_id
                )
                if not reliable:
                    votes = []
                    rate.sleep()
                    continue
                if votes and votes[-1]["yolo_class_id"] != yolo_id:
                    votes = []
                votes.append(cube)
                if len(votes) >= self.vision_quick_classify_frames:
                    selected = max(
                        votes, key=lambda item: item["confidence"]
                    )
                    self._status(
                        "Observation camera classified %s region as %s "
                        "(YOLO+template %d/%d, confidence %.3f..%.3f)"
                        % (
                            region["display_name"],
                            selected["yolo_class_name"],
                            len(votes),
                            self.vision_quick_classify_frames,
                            min(item["confidence"] for item in votes),
                            max(item["confidence"] for item in votes),
                        )
                    )
                    return selected
                rate.sleep()
        finally:
            self.cmd_pub.publish(Twist())
        self._status(
            "Observation classification uncertain in %s; "
            "using close-range verification"
            % region["display_name"]
        )
        return None

    @staticmethod
    def _inside_grasp_range(detection, region):
        acceptance = region["grasp_acceptance"]
        values = {
            "center_x": detection["center_x"],
            "center_y": detection["center_y"],
            "width": detection["width"],
            "height": detection["height"],
        }
        return all(
            acceptance[key][0] <= value <= acceptance[key][1]
            for key, value in values.items()
        )

    @staticmethod
    def _ensure_minimum_speed(value, minimum):
        if value == 0.0 or abs(value) >= minimum:
            return value
        return math.copysign(minimum, value)

    def _vision_align(self, region, initial_detection):
        target = region["grasp_target"]
        deadline = rospy.Time.now() + rospy.Duration(self.vision_align_timeout)
        last_seen = rospy.Time.now()
        last_sequence = -1
        last_horizontal_error = target[0] - initial_detection["center_x"]
        tracked_center = (
            initial_detection["center_x"],
            initial_detection["center_y"],
        )
        stable_frames = 0
        rate = rospy.Rate(20)
        self._status(
            "Visual servo active in %s region; target box=(%.3f, %.3f, %.3f, %.3f)"
            % (
                region["display_name"],
                target[0], target[1], target[2], target[3],
            )
        )
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    rate.sleep()
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if detections:
                    detection = min(
                        detections,
                        key=lambda item: (
                            (item["center_x"] - tracked_center[0]) ** 2
                            + (item["center_y"] - tracked_center[1]) ** 2
                        ),
                    )
                    tracking_distance = math.hypot(
                        detection["center_x"] - tracked_center[0],
                        detection["center_y"] - tracked_center[1],
                    )
                    if tracking_distance > 0.25:
                        detection = None
                else:
                    detection = None
                if detection is None:
                    stable_frames = 0
                    missing_for = (rospy.Time.now() - last_seen).to_sec()
                    if missing_for >= self.vision_lost_timeout:
                        self._status(
                            "Visual alignment lost the selected cube for %.1f seconds"
                            % missing_for
                        )
                        return False
                    command = Twist()
                    if missing_for >= 0.5 and last_horizontal_error != 0.0:
                        command.angular.z = math.copysign(0.10, last_horizontal_error)
                    self.cmd_pub.publish(command)
                    rate.sleep()
                    continue

                last_seen = rospy.Time.now()
                tracked_center = (
                    detection["center_x"],
                    detection["center_y"],
                )
                horizontal_error = target[0] - detection["center_x"]
                vertical_error = target[1] - detection["center_y"]
                height_error = target[3] - detection["height"]
                last_horizontal_error = horizontal_error

                if self._inside_grasp_range(detection, region):
                    stable_frames += 1
                    self.cmd_pub.publish(Twist())
                    if stable_frames >= self.vision_align_stable_frames:
                        self._status(
                            "Visual grasp range reached in %s: "
                            "box=(%.3f, %.3f, %.3f, %.3f), confidence=%.3f"
                            % (
                                region["display_name"],
                                detection["center_x"],
                                detection["center_y"],
                                detection["width"],
                                detection["height"],
                                detection["confidence"],
                            )
                        )
                        return True
                    rate.sleep()
                    continue

                stable_frames = 0
                command = Twist()
                command.angular.z = clamp(
                    self.vision_angular_gain * horizontal_error,
                    -self.vision_max_angular,
                    self.vision_max_angular,
                )
                if abs(horizontal_error) > 0.020:
                    command.angular.z = self._ensure_minimum_speed(
                        command.angular.z, 0.055
                    )
                else:
                    command.angular.z = 0.0

                if abs(horizontal_error) <= 0.080:
                    distance_error = vertical_error + 0.45 * height_error
                    command.linear.x = clamp(
                        self.vision_forward_gain * distance_error,
                        -self.vision_max_forward,
                        self.vision_max_forward,
                    )
                    if abs(distance_error) > 0.020:
                        command.linear.x = self._ensure_minimum_speed(
                            command.linear.x, 0.020
                        )
                self.cmd_pub.publish(command)
                rospy.loginfo_throttle(
                    1.0,
                    "Vision servo %s: cx=%.3f cy=%.3f w=%.3f h=%.3f "
                    "cmd=(%.3f, %.3f)"
                    % (
                        region["name"],
                        detection["center_x"],
                        detection["center_y"],
                        detection["width"],
                        detection["height"],
                        command.linear.x,
                        command.angular.z,
                    ),
                )
                rate.sleep()
        finally:
            self.cmd_pub.publish(Twist())
        self._status("Visual alignment timed out in %s region" % region["display_name"])
        return False

    def _classify_aligned_cube(self, region):
        """Classify only after the box is inside the recorded grasp range.

        The label occupies too few pixels at the observation pose, and lateral
        views introduce strong perspective distortion.  The recorded
        bottom-centre grasp view is the reliable classification domain.
        """
        deadline = rospy.Time.now() + rospy.Duration(self.vision_classify_timeout)
        last_sequence = -1
        votes = []
        rate = rospy.Rate(20)
        target_x, target_y = region["grasp_target"][:2]
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    rate.sleep()
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if not detections:
                    votes = []
                    rate.sleep()
                    continue
                cube = min(
                    detections,
                    key=lambda item: (
                        (item["center_x"] - target_x) ** 2
                        + (item["center_y"] - target_y) ** 2
                    ),
                )
                if (
                    not self._inside_grasp_range(cube, region)
                    or cube["template_class_id"] is None
                ):
                    votes = []
                    rate.sleep()
                    continue
                votes.append(cube)
                if len(votes) >= self.vision_classify_stable_frames:
                    break
                rate.sleep()
        finally:
            self.cmd_pub.publish(Twist())

        if len(votes) < self.vision_classify_stable_frames:
            self._status(
                "Aligned classification timed out in %s region"
                % region["display_name"]
            )
            return None

        counts = {}
        raw_counts = {}
        for vote in votes:
            template_id = vote["template_class_id"]
            counts[template_id] = counts.get(template_id, 0) + 1
            raw_id = vote["yolo_class_id"]
            raw_counts[raw_id] = raw_counts.get(raw_id, 0) + 1
        voted_class_id, support = max(
            counts.items(), key=lambda item: item[1]
        )
        if support <= len(votes) // 2:
            self._status(
                "Aligned classification in %s has no majority: %s"
                % (region["display_name"], counts)
            )
            return None
        voted_cube = max(
            (
                vote for vote in votes
                if vote["template_class_id"] == voted_class_id
            ),
            key=lambda vote: vote["template_score"],
        )
        raw_class_id, raw_support = max(
            raw_counts.items(), key=lambda item: item[1]
        )
        self._status(
            "Grasp-view camera classified %s region as %s "
            "(template vote=%d/%d score=%.3f margin=%.3f; "
            "YOLO raw=%s %d/%d)"
            % (
                region["display_name"],
                voted_cube["template_class_name"],
                support,
                len(votes),
                voted_cube["template_score"],
                voted_cube["template_margin"],
                self.vision_class_names[raw_class_id],
                raw_support,
                len(votes),
            )
        )
        return voted_cube

    def _find_and_align_target(self):
        for region in self.search_regions:
            if not self._move_base(
                region["observation_goal"],
                "%s visual observation pose" % region["display_name"],
            ):
                self._status(
                    "Cannot reach %s observation pose; continuing search"
                    % region["display_name"]
                )
                continue
            # move_base can keep publishing for a short time after reporting
            # success.  Relinquish navigation before camera steering starts.
            self.nav.cancel_all_goals()
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.20)
            self._status(
                "Scanning %s region for %s with YOLOv5"
                % (region["display_name"], self.category)
            )
            detection = self._scan_region(region)
            if detection is None:
                continue
            quick_classified = self._quick_classify_observation(
                region, detection
            )
            if (
                quick_classified is not None
                and quick_classified["class_id"] != self.target_class_id
            ):
                self._status(
                    "%s region is confidently not %s at the observation pose; "
                    "skipping close approach"
                    % (region["display_name"], self.category)
                )
                continue
            if quick_classified is not None:
                detection = quick_classified
                self._status(
                    "%s region may contain %s; confirming at grasp range"
                    % (region["display_name"], self.category)
                )
            if not self._vision_align(region, detection):
                self._status(
                    "Could not visually align the cube in %s; continuing search"
                    % region["display_name"]
                )
                continue
            classified = self._classify_aligned_cube(region)
            if classified is None:
                continue
            if classified["class_id"] != self.target_class_id:
                self._status(
                    "%s region is not %s; continuing search"
                    % (region["display_name"], self.category)
                )
                continue
            self._status(
                "Camera recognised %s in %s region at the calibrated grasp view"
                % (self.category, region["display_name"])
            )
            return region
        raise rospy.ROSException(
            "YOLOv5 could not find category %s in left/middle/right regions"
            % self.category
        )

    def _move_base(self, goal, description):
        message = MoveBaseGoal()
        message.target_pose.header.frame_id = "map"
        message.target_pose.header.stamp = rospy.Time.now()
        message.target_pose.pose.position.x = goal[0]
        message.target_pose.pose.position.y = goal[1]
        message.target_pose.pose.orientation = quaternion_from_yaw(goal[2])
        self._status(
            "Navigating to %s: (%.4f, %.4f, %.4f)" % (description, goal[0], goal[1], goal[2])
        )
        for attempt in range(1, 4):
            self.nav.send_goal(message)
            if self.nav.wait_for_result(rospy.Duration(self.nav_timeout)):
                if self.nav.get_state() == GoalStatus.SUCCEEDED:
                    return True
                self._status("move_base attempt %d returned state %d; retrying" % (attempt, self.nav.get_state()))
            else:
                self._status("move_base attempt %d timed out; retrying" % attempt)
            self.nav.cancel_all_goals()
            try:
                self.clear_costmaps()
            except rospy.ServiceException:
                pass
            rospy.sleep(1.0)
        return False

    def _start_arm_control(self):
        # Stop the Gazebo-only parked-pose holder before a real controller
        # claims the same joints.  Publishing more than once avoids a lost
        # transient connection when task3_execute is launched in a new shell.
        for _ in range(3):
            self.arm_control_enabled_pub.publish(Bool(data=True))
            rospy.sleep(0.08)
        rospy.sleep(0.25)

        for _ in range(12):
            states = {
                controller.name: controller.state
                for controller in self.list_controllers().controller
            }
            needed = [
                name for name in ("arm_controller", "gripper_controller")
                if states.get(name) != "running"
            ]
            if not needed:
                self._status("Pickup pose reached: arm/gripper controllers enabled")
                return
            request = SwitchControllerRequest()
            request.start_controllers = needed
            request.strictness = SwitchControllerRequest.STRICT
            request.start_asap = True
            request.timeout = 2.0
            if self.switch_controllers(request).ok:
                rospy.sleep(0.15)
                continue
            rospy.sleep(0.20)
        states = {
            controller.name: controller.state
            for controller in self.list_controllers().controller
        }
        raise rospy.ROSException(
            "arm/gripper controllers could not be started; states=%s" % states
        )

    def _move_arm(self, positions, duration):
        message = JointTrajectory()
        message.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = rospy.Duration(duration)
        message.points = [point]
        for _ in range(3):
            self.arm_pub.publish(message)
            rospy.sleep(0.05)
        rospy.sleep(duration + 0.20)

    def _set_gripper(self, position):
        for _ in range(3):
            self.gripper_pub.publish(Float64(data=position))
            rospy.sleep(0.08)

    def _wait_for_grasp_state(self, wanted, timeout):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.grasp_state == wanted:
                return True
            rospy.sleep(0.05)
        return self.grasp_state == wanted

    def _set_cargo_to_tcp(self):
        tcp = self.get_link("car3::tcp_link", "world")
        if not tcp.success:
            raise rospy.ROSException("could not read TCP pose")
        state = ModelState()
        state.model_name = self.cargo_model
        state.pose = tcp.link_state.pose
        state.twist = Twist()
        state.reference_frame = "world"
        if not self.set_model(state).success:
            raise rospy.ROSException("could not align target cube with TCP")

    def _fallback_hold_tick(self, _event):
        if not self.fallback_holding:
            return
        try:
            self._set_cargo_to_tcp()
        except rospy.ServiceException:
            pass

    def _pick(self):
        self._status("Opening gripper and moving camera/gripper to the visually aligned cube")
        self._set_gripper(1.0)
        self._wait_for_grasp_state("IDLE", 1.0)
        self._move_arm(self.arm_grasp, self.arm_grasp_duration)
        self._set_gripper(0.76)
        if self._wait_for_grasp_state("GRASPING", 1.5):
            self._status("Real grasp_attach pickup succeeded")
        else:
            # Permitted simulation fallback: only after a genuine close has
            # failed, place the already-recognised target at the TCP and close
            # again so grasp_attach can bind it.
            self._status("Physical overlap was not detected; using final TCP attachment fallback")
            self._set_gripper(1.0)
            rospy.sleep(0.30)
            self._set_cargo_to_tcp()
            self._set_gripper(0.76)
            if not self._wait_for_grasp_state("GRASPING", 1.5):
                self.fallback_holding = True
                self.fallback_hold_timer = rospy.Timer(
                    rospy.Duration(1.0 / 30.0), self._fallback_hold_tick
                )
                self._status("Fallback hold enabled; keeping recognised target at TCP")
            else:
                self._status("TCP attachment fallback succeeded")
        # This is the user-calibrated carry posture; the gripper remains closed.
        self._move_arm(self.arm_carry, self.arm_carry_duration)
        self.carry_mode_pub.publish(Bool(data=True))
        self._status("Cargo is held; arm is in carry pose; factory navigation speed is 80%")

    def run(self):
        self._wait_for_services()
        region = self._find_and_align_target()
        self._status(
            "%s was selected from camera recognition in %s region"
            % (self.cargo_name, region["display_name"])
        )
        self._start_arm_control()
        self._pick()
        if not self._move_base(self.destination, self.destination_name):
            raise rospy.ROSException("cannot reach %s" % self.destination_name)
        self.cmd_pub.publish(Twist())
        result = "%s delivered to %s; gripper remains closed" % (self.cargo_name, self.destination_name)
        self._status("DONE: " + result)
        self.done_pub.publish(Bool(data=True))
        rospy.spin()


if __name__ == "__main__":
    try:
        PickDeliverTask().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("task3_pick_deliver failed: %s", error)
        raise
