#!/usr/bin/env python3
"""Capture a three-class YOLOv5 dataset with the task arm kept stationary.

This is a simulation data-generation tool, not the final perception pipeline.
Gazebo model poses are deliberately retained here for class ground truth and
for reaching each calibrated pickup bay.  Once the vehicle arrives, the arm
remains in the prepared initial posture while small base distance/yaw changes
place the target cube in a physical 3x3 set of camera locations.
"""

import json
import math
import os
import sys
import threading
import time

import cv2
import rospy
import tf.transformations as transformations
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool

import rospkg


# Reuse the task's calibrated navigation and fine-alignment helpers so dataset
# collection observes the exact pose used immediately before a real pickup.
SCRIPTS_DIR = os.path.join(rospkg.RosPack().get_path("car3"), "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from task3_pick_deliver import (  # noqa: E402
    ALL_CUBES,
    ARM_JOINTS,
    LEFT_REFERENCE_YAW,
    LEFT_TARGET_MINUS_CAR,
    PickDeliverTask,
    rotate_2d,
    wrap,
)


CLASS_BY_MODEL = {"cube_0": 0, "cube_1": 1, "cube_2": 2}
CLASS_NAMES = ("food", "daily", "electronics")

# Capture the real arrival view first.  The remaining entries are ordered
# row-major so each class can also produce an immediately readable preview.
GRID_CELLS = (
    ("middle_center", 1, 1),
    ("top_left", 0, 0),
    ("top_center", 0, 1),
    ("top_right", 0, 2),
    ("middle_left", 1, 0),
    ("middle_right", 1, 2),
    ("bottom_left", 2, 0),
    ("bottom_center", 2, 1),
    ("bottom_right", 2, 2),
)


def _float_list_param(name, default, expected_length):
    values = rospy.get_param(name, default)
    if isinstance(values, str):
        values = [value.strip() for value in values.split(",") if value.strip()]
    if not isinstance(values, (list, tuple)) or len(values) != expected_length:
        raise rospy.ROSException(
            "%s must contain %d numeric values" % (name, expected_length)
        )
    return [float(value) for value in values]


def _string_list_param(name, default):
    values = rospy.get_param(name, default)
    if isinstance(values, str):
        values = [value.strip() for value in values.split(",") if value.strip()]
    if not isinstance(values, (list, tuple)):
        raise rospy.ROSException("%s must be a list or comma-separated string" % name)
    return set(str(value) for value in values)


class CubeDatasetCapture:
    def __init__(self):
        # PickDeliverTask calls rospy.init_node and supplies calibrated movement
        # helpers.  cargo_category only satisfies its constructor; all three
        # models are captured below.
        self.task = PickDeliverTask()
        self.dataset_dir = os.path.abspath(os.path.expanduser(
            rospy.get_param("~dataset_dir", "~/smartcar-yolo-dataset")
        ))
        self.frames_per_cell = max(
            1, int(rospy.get_param("~frames_per_cell", 1))
        )
        self.frame_interval = max(
            0.05, float(rospy.get_param("~frame_interval", 0.20))
        )
        self.settle_seconds = max(
            0.10, float(rospy.get_param("~settle_seconds", 0.60))
        )
        self.camera_timeout = max(
            0.50, float(rospy.get_param("~camera_timeout", 5.0))
        )
        self.arm_ready_timeout = max(
            1.0, float(rospy.get_param("~arm_ready_timeout", 15.0))
        )
        self.arm_stationary_tolerance = max(
            0.001, float(rospy.get_param("~arm_stationary_tolerance", 0.03))
        )
        self.direct_positioning = bool(
            rospy.get_param("~direct_positioning", False)
        )

        # Moving backward from the calibrated pickup pose raises the object in
        # this arm-mounted camera.  Small yaw offsets move it left/right.  The
        # values were measured with the arm in task3_prepare's initial posture.
        self.grid_distance_offsets = _float_list_param(
            "~grid_distance_offsets", [0.25, 0.10, 0.0], 3
        )
        self.grid_yaw_offsets = _float_list_param(
            "~grid_yaw_offsets", [-0.30, 0.0, 0.30], 3
        )
        self.validation_cells = _string_list_param(
            "~validation_cells", ["top_right", "bottom_left"]
        )
        unknown_validation_cells = self.validation_cells.difference(
            cell_name for cell_name, _, _ in GRID_CELLS
        )
        if unknown_validation_cells:
            raise rospy.ROSException(
                "unknown validation grid cells: %s"
                % sorted(unknown_validation_cells)
            )

        self.bbox_expand_x = max(
            0.0, float(rospy.get_param("~bbox_expand_x", 0.08))
        )
        self.bbox_expand_top = max(
            0.0, float(rospy.get_param("~bbox_expand_top", 0.05))
        )
        self.bbox_expand_bottom = max(
            0.0, float(rospy.get_param("~bbox_expand_bottom", 0.40))
        )

        self.bridge = CvBridge()
        self.get_model = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        self.image_lock = threading.Lock()
        self.latest_image = None
        self.latest_image_sequence = 0
        self.camera_info = None
        self.run_id = time.strftime("%Y%m%d_%H%M%S")
        self.initial_arm_positions = None
        self.saved_records = []

        camera_topic = rospy.get_param(
            "~camera_topic", "/camera/rgb/image_raw"
        )
        camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/rgb/camera_info"
        )
        rospy.Subscriber(
            camera_topic, Image, self._image_callback, queue_size=1
        )
        rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )

        self.image_dirs = {
            split: os.path.join(self.dataset_dir, "images", split)
            for split in ("train", "val")
        }
        self.label_dirs = {
            split: os.path.join(self.dataset_dir, "labels", split)
            for split in ("train", "val")
        }
        self.previews_dir = os.path.join(self.dataset_dir, "previews")
        for directory in (
            list(self.image_dirs.values())
            + list(self.label_dirs.values())
            + [self.previews_dir]
        ):
            os.makedirs(directory, exist_ok=True)

        self.metadata_path = os.path.join(self.dataset_dir, "metadata.jsonl")
        self._write_dataset_files()

    def _write_dataset_files(self):
        with open(
            os.path.join(self.dataset_dir, "classes.txt"),
            "w",
            encoding="utf-8",
        ) as handle:
            handle.write("\n".join(CLASS_NAMES) + "\n")

        with open(
            os.path.join(self.dataset_dir, "dataset.yaml"),
            "w",
            encoding="utf-8",
        ) as handle:
            handle.write("path: .\n")
            handle.write("train: images/train\n")
            handle.write("val: images/val\n")
            handle.write("nc: %d\n" % len(CLASS_NAMES))
            handle.write("names:\n")
            for class_id, class_name in enumerate(CLASS_NAMES):
                handle.write("  %d: %s\n" % (class_id, class_name))

    def _image_callback(self, message):
        with self.image_lock:
            self.latest_image = message
            self.latest_image_sequence += 1

    def _camera_info_callback(self, message):
        self.camera_info = message

    def _wait_for_camera(self):
        deadline = time.monotonic() + self.camera_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.image_lock:
                have_image = self.latest_image is not None
            if have_image and self.camera_info is not None:
                return
            rospy.sleep(0.05)
        raise rospy.ROSException("camera image or camera_info did not arrive")

    def _wait_for_fresh_image(self, after_sequence):
        deadline = time.monotonic() + self.camera_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.image_lock:
                if (
                    self.latest_image is not None
                    and self.latest_image_sequence > after_sequence
                ):
                    return self.latest_image, self.latest_image_sequence
            rospy.sleep(0.02)
        raise rospy.ROSException("a fresh RGB camera frame did not arrive")

    @staticmethod
    def _arm_positions_from_message(message):
        positions = {}
        for joint_name in ARM_JOINTS:
            if joint_name not in message.name:
                raise rospy.ROSException(
                    "joint_states is missing %s" % joint_name
                )
            positions[joint_name] = message.position[
                message.name.index(joint_name)
            ]
        return positions

    def _wait_for_stationary_arm(self):
        ready = rospy.wait_for_message(
            "/sim_task3/arm_initial_pose_ready",
            Bool,
            timeout=self.arm_ready_timeout,
        )
        if not ready.data:
            raise rospy.ROSException("prepared initial arm pose is not ready")
        joint_state = rospy.wait_for_message(
            "/joint_states", JointState, timeout=self.arm_ready_timeout
        )
        self.initial_arm_positions = self._arm_positions_from_message(
            joint_state
        )
        self.task._status(
            "Dataset capture locked the stationary arm baseline: %s"
            % [
                round(self.initial_arm_positions[joint], 4)
                for joint in ARM_JOINTS
            ]
        )

    def _assert_arm_stationary(self):
        message = rospy.wait_for_message(
            "/joint_states", JointState, timeout=self.arm_ready_timeout
        )
        current = self._arm_positions_from_message(message)
        largest_drift = max(
            abs(current[joint] - self.initial_arm_positions[joint])
            for joint in ARM_JOINTS
        )
        if largest_drift > self.arm_stationary_tolerance:
            raise rospy.ROSException(
                "arm moved during dataset capture: max drift %.4f rad exceeds %.4f"
                % (largest_drift, self.arm_stationary_tolerance)
            )
        return current

    def _cube_poses(self):
        poses = {}
        for model in ALL_CUBES:
            response = self.get_model(model, "world")
            if not response.success:
                raise rospy.ROSException(
                    "target model %s has not spawned" % model
                )
            poses[model] = response.pose
        return poses

    def _set_base_pose(self, goal, description):
        quaternion = transformations.quaternion_from_euler(
            0.0, 0.0, goal[2]
        )
        state = ModelState()
        state.model_name = "car3"
        state.pose = Pose(
            position=Point(x=goal[0], y=goal[1], z=0.01),
            orientation=Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        )
        state.twist = Twist()
        state.reference_frame = "world"
        response = self.task.set_model(state)
        if not response.success:
            rospy.logerr(
                "Could not position vehicle for %s: %s",
                description,
                response.status_message,
            )
            return False
        rospy.sleep(self.settle_seconds)
        return True

    def _reach_pickup_pose(self, goal, description):
        if self.direct_positioning:
            return self._set_base_pose(goal, description)
        if not self.task._move_base(goal, description):
            return False
        return self.task._fine_align_base(goal)

    def _reach_grid_pose(self, goal, description):
        if self.direct_positioning:
            return self._set_base_pose(goal, description)
        # These are small movements around an already reached pickup bay.
        # Fine alignment drives the real planar base at low speed while the
        # prepared arm-pose holder remains active.
        return self.task._fine_align_base(goal)

    @staticmethod
    def _pickup_goal(model, poses):
        ordered = sorted(
            ALL_CUBES, key=lambda name: poses[name].position.x
        )
        region_index = ordered.index(model)
        region_name = ("left", "upper", "right")[region_index]
        pickup_yaw = wrap(
            LEFT_REFERENCE_YAW - region_index * math.pi / 2.0
        )
        local_dx, local_dy = rotate_2d(
            LEFT_TARGET_MINUS_CAR[0],
            LEFT_TARGET_MINUS_CAR[1],
            -LEFT_REFERENCE_YAW,
        )
        target_minus_car_x, target_minus_car_y = rotate_2d(
            local_dx, local_dy, pickup_yaw
        )
        target = poses[model].position
        return (
            target.x - target_minus_car_x,
            target.y - target_minus_car_y,
            pickup_yaw,
        ), region_name

    def _grid_goal(self, pickup_goal, row, column):
        # Positive distance offset moves backward from the cube, which raises
        # it in the image while keeping the arm and camera joints unchanged.
        backward_x, backward_y = rotate_2d(
            -self.grid_distance_offsets[row], 0.0, pickup_goal[2]
        )
        return (
            pickup_goal[0] + backward_x,
            pickup_goal[1] + backward_y,
            wrap(pickup_goal[2] + self.grid_yaw_offsets[column]),
        )

    @staticmethod
    def _expected_cell_centre(row, column):
        return (
            (column + 0.5) / 3.0,
            (row + 0.5) / 3.0,
        )

    @staticmethod
    def _detect_cube_near(image, expected_centre):
        """Return the bright labelled cube face nearest a grid cell centre."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        height, width = gray.shape
        expected_x = expected_centre[0] * width
        expected_y = expected_centre[1] * height
        candidates = []
        for contour in contours:
            left, top, box_width, box_height = cv2.boundingRect(contour)
            area = box_width * box_height
            if box_width < 10 or box_height < 10 or area < 180:
                continue
            if box_width > width * 0.80 or box_height > height * 0.80:
                continue
            centre_x = left + box_width * 0.5
            centre_y = top + box_height * 0.5
            distance = math.hypot(
                (centre_x - expected_x) / width,
                (centre_y - expected_y) / height,
            )
            candidates.append((
                distance,
                float(left),
                float(top),
                float(left + box_width),
                float(top + box_height),
            ))
        if not candidates:
            return None
        _, left, top, right, bottom = min(candidates)
        return left, top, right, bottom

    def _expanded_box(self, box, image):
        left, top, right, bottom = box
        box_width = right - left
        box_height = bottom - top
        image_height, image_width = image.shape[:2]
        return (
            max(0.0, left - box_width * self.bbox_expand_x),
            max(0.0, top - box_height * self.bbox_expand_top),
            min(
                float(image_width - 1),
                right + box_width * self.bbox_expand_x,
            ),
            min(
                float(image_height - 1),
                bottom + box_height * self.bbox_expand_bottom,
            ),
        )

    def _label_row(self, target_model, box, image):
        left, top, right, bottom = box
        width = float(image.shape[1])
        height = float(image.shape[0])
        centre_x = (left + right) * 0.5 / width
        centre_y = (top + bottom) * 0.5 / height
        box_width = (right - left) / width
        box_height = (bottom - top) / height
        return "%d %.6f %.6f %.6f %.6f" % (
            CLASS_BY_MODEL[target_model],
            centre_x,
            centre_y,
            box_width,
            box_height,
        )

    def _capture_frame(
        self,
        target_model,
        target_region,
        grid_cell,
        row,
        column,
        grid_goal,
        frame_index,
        after_sequence,
    ):
        message, sequence = self._wait_for_fresh_image(after_sequence)
        try:
            image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            raise rospy.ROSException(
                "could not convert camera image: %s" % error
            )

        expected_centre = self._expected_cell_centre(row, column)
        detected_box = self._detect_cube_near(image, expected_centre)
        if detected_box is None:
            raise rospy.ROSException(
                "could not locate %s in grid cell %s"
                % (target_model, grid_cell)
            )
        box = self._expanded_box(detected_box, image)
        split = "val" if grid_cell in self.validation_cells else "train"
        filename = "%s_%s_%s_%s_%02d" % (
            self.run_id,
            target_region,
            target_model,
            grid_cell,
            frame_index,
        )
        image_path = os.path.join(
            self.image_dirs[split], filename + ".jpg"
        )
        label_path = os.path.join(
            self.label_dirs[split], filename + ".txt"
        )
        if not cv2.imwrite(
            image_path, image, [cv2.IMWRITE_JPEG_QUALITY, 95]
        ):
            raise rospy.ROSException("could not write %s" % image_path)
        label_row = self._label_row(target_model, box, image)
        with open(label_path, "w", encoding="utf-8") as handle:
            handle.write(label_row + "\n")

        arm_positions = self._assert_arm_stationary()
        live_poses = self._cube_poses()
        record = {
            "run_id": self.run_id,
            "image": os.path.relpath(image_path, self.dataset_dir),
            "label": os.path.relpath(label_path, self.dataset_dir),
            "split": split,
            "target_model": target_model,
            "class_id": CLASS_BY_MODEL[target_model],
            "class_name": CLASS_NAMES[CLASS_BY_MODEL[target_model]],
            "target_region": target_region,
            "grid_cell": grid_cell,
            "grid_row": row,
            "grid_column": column,
            "expected_centre_normalized": expected_centre,
            "bbox_px": box,
            "bbox_source": "rgb_bright_label_contour",
            "sim_time": message.header.stamp.to_sec(),
            "base_goal": {
                "x": grid_goal[0],
                "y": grid_goal[1],
                "yaw": grid_goal[2],
            },
            "arm_positions": {
                joint: arm_positions[joint] for joint in ARM_JOINTS
            },
            "cube_world_poses": {
                model: {
                    "x": live_poses[model].position.x,
                    "y": live_poses[model].position.y,
                    "z": live_poses[model].position.z,
                }
                for model in ALL_CUBES
            },
        }
        with open(self.metadata_path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.saved_records.append(record)
        rospy.loginfo(
            "Saved %s [%s, class=%s, cell=%s]",
            image_path,
            split,
            record["class_name"],
            grid_cell,
        )
        return sequence, image_path, box

    def _write_grid_preview(self, target_model, target_region, samples):
        preview_rows = []
        for row in range(3):
            preview_columns = []
            for column in range(3):
                sample = samples[(row, column)]
                image = cv2.imread(sample["image_path"])
                if image is None:
                    raise rospy.ROSException(
                        "could not read preview source %s"
                        % sample["image_path"]
                    )
                left, top, right, bottom = sample["bbox"]
                cv2.rectangle(
                    image,
                    (int(round(left)), int(round(top))),
                    (int(round(right)), int(round(bottom))),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    image,
                    sample["grid_cell"],
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                preview_columns.append(
                    cv2.resize(image, (320, 240))
                )
            preview_rows.append(cv2.hconcat(preview_columns))
        preview = cv2.vconcat(preview_rows)
        preview_path = os.path.join(
            self.previews_dir,
            "%s_%s_%s_grid.jpg"
            % (self.run_id, target_region, target_model),
        )
        if not cv2.imwrite(
            preview_path, preview, [cv2.IMWRITE_JPEG_QUALITY, 92]
        ):
            raise rospy.ROSException(
                "could not write preview %s" % preview_path
            )
        rospy.loginfo("Saved 3x3 preview %s", preview_path)

    def _write_summary(self):
        counts = {
            split: {
                class_name: sum(
                    1
                    for record in self.saved_records
                    if record["split"] == split
                    and record["class_name"] == class_name
                )
                for class_name in CLASS_NAMES
            }
            for split in ("train", "val")
        }
        summary = {
            "run_id": self.run_id,
            "dataset_dir": ".",
            "classes": list(CLASS_NAMES),
            "counts": counts,
            "total_images": len(self.saved_records),
            "arm_remained_stationary": True,
            "grid_distance_offsets": self.grid_distance_offsets,
            "grid_yaw_offsets": self.grid_yaw_offsets,
        }
        with open(
            os.path.join(self.dataset_dir, "summary.json"),
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)
            handle.write("\n")

    def run(self):
        self.task._wait_for_services()
        self._wait_for_camera()
        self._wait_for_stationary_arm()
        poses = self._cube_poses()
        ordered = sorted(
            ALL_CUBES, key=lambda model: poses[model].position.x
        )

        for model in ordered:
            poses = self._cube_poses()
            pickup_goal, region = self._pickup_goal(model, poses)
            transport = (
                "direct positioning"
                if self.direct_positioning
                else "navigation"
            )
            self.task._status(
                "Dataset capture: %s to %s (%s), arm remains stationary"
                % (transport, model, region)
            )
            if not self._reach_pickup_pose(
                pickup_goal, "%s dataset capture" % region
            ):
                raise rospy.ROSException(
                    "cannot reach %s capture pose" % region
                )
            self._assert_arm_stationary()

            preview_samples = {}
            for grid_cell, row, column in GRID_CELLS:
                grid_goal = self._grid_goal(
                    pickup_goal, row, column
                )
                self.task._status(
                    "Dataset %s/%s: moving stationary-arm camera view to %s"
                    % (model, region, grid_cell)
                )
                if not self._reach_grid_pose(
                    grid_goal, "%s %s" % (model, grid_cell)
                ):
                    raise rospy.ROSException(
                        "cannot reach grid pose %s for %s"
                        % (grid_cell, model)
                    )
                self._assert_arm_stationary()
                rospy.sleep(self.settle_seconds)
                with self.image_lock:
                    sequence = self.latest_image_sequence
                for frame_index in range(self.frames_per_cell):
                    sequence, image_path, box = self._capture_frame(
                        model,
                        region,
                        grid_cell,
                        row,
                        column,
                        grid_goal,
                        frame_index,
                        sequence,
                    )
                    if frame_index == 0:
                        preview_samples[(row, column)] = {
                            "grid_cell": grid_cell,
                            "image_path": image_path,
                            "bbox": box,
                        }
                    rospy.sleep(self.frame_interval)
            self._write_grid_preview(
                model, region, preview_samples
            )

        self._write_summary()
        self.task._status(
            "DATASET DONE: %d images and YOLOv5 labels saved to %s; "
            "arm remained stationary"
            % (len(self.saved_records), self.dataset_dir)
        )


if __name__ == "__main__":
    try:
        CubeDatasetCapture().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("capture_cube_dataset failed: %s", error)
        raise
