#!/usr/bin/env python3
"""Capture labelled RGB images of all randomised task cubes for YOLO training.

Run this only after ``task3_prepare.launch``.  The node drives to each live
cube using the same calibrated pickup projection as the task, puts the arm in
the grasp pose, saves RGB frames, and writes one YOLO label file per image.
"""

import json
import math
import os
import threading
import time

import cv2
import rospy
import tf.transformations as transformations
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from sensor_msgs.msg import CameraInfo, Image

# Reuse the task's calibrated navigation, fine alignment, controller, and arm
# helpers so data collection views the same scene geometry as a real pickup.
import rospkg
import sys

SCRIPTS_DIR = os.path.join(rospkg.RosPack().get_path("car3"), "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from task3_pick_deliver import (  # noqa: E402
    ALL_CUBES,
    LEFT_REFERENCE_YAW,
    LEFT_TARGET_MINUS_CAR,
    PickDeliverTask,
    rotate_2d,
    wrap,
)


CLASS_BY_MODEL = {"cube_0": 0, "cube_1": 1, "cube_2": 2}
CLASS_NAMES = ("food", "daily", "electronics")
CUBE_HALF_SIZE_M = 0.02


class CubeDatasetCapture:
    def __init__(self):
        # PickDeliverTask calls rospy.init_node and supplies the calibrated
        # navigation/arm primitives.  The launch file sets cargo_category so
        # its category resolver has a valid default.
        self.task = PickDeliverTask()
        self.dataset_dir = os.path.expanduser(
            rospy.get_param("~dataset_dir", "~/smartcar-yolo-dataset")
        )
        self.frames_per_cube = max(1, int(rospy.get_param("~frames_per_cube", 3)))
        self.frame_interval = max(0.05, float(rospy.get_param("~frame_interval", 0.25)))
        self.settle_seconds = max(0.0, float(rospy.get_param("~settle_seconds", 0.8)))
        self.camera_timeout = max(0.5, float(rospy.get_param("~camera_timeout", 5.0)))
        self.max_camera_adjustment = max(
            0.05, float(rospy.get_param("~max_camera_adjustment", 0.80))
        )
        self.direct_positioning = bool(rospy.get_param("~direct_positioning", True))
        # The contact grasp pose puts the camera almost inside a 4 cm cube.
        # This calibrated pre-grasp observation pose keeps the cube fully in view.
        self.arm_capture = self.task._pose_param(
            "~arm_capture_pose", [-0.0001, 1.1000, 0.7000, 1.2000, 0.0]
        )
        self.arm_capture_duration = max(
            0.2, float(rospy.get_param("~arm_capture_duration", 1.5))
        )
        self.model_frame = rospy.get_param("~model_frame", "odom")
        self.park_pose = self.task._pose_param(
            "~arm_park_pose", [-0.0001, -0.4999, 1.2800, 1.7000, 0.0000]
        )

        self.bridge = CvBridge()
        self.get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.image_lock = threading.Lock()
        self.latest_image = None
        self.latest_image_stamp = rospy.Time(0)
        self.camera_info = None
        self.image_sequence = 0
        self.run_id = time.strftime("%Y%m%d_%H%M%S")

        camera_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/rgb/camera_info")
        rospy.Subscriber(camera_topic, Image, self._image_callback, queue_size=1)
        rospy.Subscriber(camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1)

        self.images_dir = os.path.join(self.dataset_dir, "images")
        self.labels_dir = os.path.join(self.dataset_dir, "labels")
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        with open(os.path.join(self.dataset_dir, "classes.txt"), "w", encoding="utf-8") as handle:
            handle.write("\n".join(CLASS_NAMES) + "\n")
        self.metadata_path = os.path.join(self.dataset_dir, "metadata.jsonl")

    def _image_callback(self, message):
        with self.image_lock:
            self.latest_image = message
            self.latest_image_stamp = message.header.stamp

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

    def _cube_poses(self):
        poses = {}
        for model in ALL_CUBES:
            response = self.get_model(model, "world")
            if not response.success:
                raise rospy.ROSException("target model %s has not spawned" % model)
            poses[model] = response.pose
        return poses

    def _position_base(self, goal, description):
        if not self.direct_positioning:
            if not self.task._move_base(goal, description):
                return False
            return self.task._fine_align_base(goal)

        quaternion = transformations.quaternion_from_euler(0.0, 0.0, goal[2])
        state = ModelState()
        state.model_name = "car3"
        state.pose = Pose(
            position=Point(x=goal[0], y=goal[1], z=0.01),
            orientation=Quaternion(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            ),
        )
        state.twist = Twist()
        state.reference_frame = "world"
        response = self.task.set_model(state)
        if not response.success:
            rospy.logerr("Could not position vehicle for %s: %s", description, response.status_message)
            return False
        rospy.sleep(0.5)
        return True

    @staticmethod
    def _pickup_goal(model, poses):
        ordered = sorted(ALL_CUBES, key=lambda name: poses[name].position.x)
        region_index = ordered.index(model)
        region_name = ("left", "upper", "right")[region_index]
        pickup_yaw = wrap(LEFT_REFERENCE_YAW - region_index * math.pi / 2.0)
        local_dx, local_dy = rotate_2d(
            LEFT_TARGET_MINUS_CAR[0], LEFT_TARGET_MINUS_CAR[1], -LEFT_REFERENCE_YAW
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

    @staticmethod
    def _corners(pose):
        matrix = transformations.quaternion_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        for x in (-CUBE_HALF_SIZE_M, CUBE_HALF_SIZE_M):
            for y in (-CUBE_HALF_SIZE_M, CUBE_HALF_SIZE_M):
                for z in (-CUBE_HALF_SIZE_M, CUBE_HALF_SIZE_M):
                    point = matrix.dot([x, y, z, 1.0])
                    yield Point(
                        x=point[0] + pose.position.x,
                        y=point[1] + pose.position.y,
                        z=point[2] + pose.position.z,
                    )

    def _project_box(self, pose):
        if self.camera_info is None:
            return None
        camera_pose = self._camera_world_pose()
        if camera_pose is None:
            return None

        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]
        image_width, image_height = self.camera_info.width, self.camera_info.height
        pixels = []
        for point in self._corners(pose):
            point_camera = self._world_to_camera(point, camera_pose)
            if point_camera.z <= 0.02:
                continue
            pixels.append((
                fx * point_camera.x / point_camera.z + cx,
                fy * point_camera.y / point_camera.z + cy,
            ))
        if len(pixels) < 4:
            return None

        left = max(0.0, min(pixel[0] for pixel in pixels))
        right = min(float(image_width - 1), max(pixel[0] for pixel in pixels))
        top = max(0.0, min(pixel[1] for pixel in pixels))
        bottom = min(float(image_height - 1), max(pixel[1] for pixel in pixels))
        if right - left < 2.0 or bottom - top < 2.0:
            return None
        return left, top, right, bottom

    def _camera_world_pose(self):
        response = self.task.get_link("car3::camera_depth_optical_frame", "world")
        if not response.success:
            rospy.logwarn("could not read Gazebo camera optical frame: %s", response.status_message)
            return None
        return response.link_state.pose

    @staticmethod
    def _world_to_camera(point, camera_pose):
        rotation_world_from_camera = transformations.quaternion_matrix([
            camera_pose.orientation.x, camera_pose.orientation.y,
            camera_pose.orientation.z, camera_pose.orientation.w,
        ])[:3, :3]
        relative_world = [
            point.x - camera_pose.position.x,
            point.y - camera_pose.position.y,
            point.z - camera_pose.position.z,
        ]
        point_camera = rotation_world_from_camera.T.dot(relative_world)
        return Point(x=point_camera[0], y=point_camera[1], z=point_camera[2])

    def _target_camera_point(self, pose):
        camera_pose = self._camera_world_pose()
        if camera_pose is None:
            return None
        point_camera = self._world_to_camera(pose.position, camera_pose)
        rotation_world_from_camera = transformations.quaternion_matrix([
            camera_pose.orientation.x, camera_pose.orientation.y,
            camera_pose.orientation.z, camera_pose.orientation.w,
        ])[:3, :3]
        return point_camera, rotation_world_from_camera.T

    def _centre_camera_view(self, pose):
        """Translate the base slowly so the grasp-pose camera sees the cube.

        The camera observes from a pre-grasp pose above the object.  This
        additional XY-only correction keeps the calibrated heading and makes
        the target centre the RGB image without changing camera height.
        """
        for _ in range(2):
            transformed = self._target_camera_point(pose)
            if transformed is None:
                return False
            point_camera, rotation_camera_from_world = transformed
            if abs(point_camera.x) <= 0.015 and abs(point_camera.y) <= 0.015:
                return True

            # p_camera(new) = p_camera(now) - R_camera_from_world * delta_xy.
            a, b = rotation_camera_from_world[0, 0], rotation_camera_from_world[0, 1]
            c, d = rotation_camera_from_world[1, 0], rotation_camera_from_world[1, 1]
            determinant = a * d - b * c
            if abs(determinant) < 1e-6:
                rospy.logwarn("Camera XY adjustment is singular; cannot centre capture target")
                return False
            rhs_x = point_camera.x
            rhs_y = point_camera.y
            delta_x = (rhs_x * d - b * rhs_y) / determinant
            delta_y = (a * rhs_y - rhs_x * c) / determinant
            adjustment = math.hypot(delta_x, delta_y)
            if adjustment > self.max_camera_adjustment:
                rospy.logwarn(
                    "Camera target adjustment %.3f m exceeds %.3f m; keeping calibrated pickup pose",
                    adjustment, self.max_camera_adjustment,
                )
                return False

            robot_pose = self.task._robot_pose()
            robot_yaw = transformations.euler_from_quaternion([
                robot_pose.orientation.x, robot_pose.orientation.y,
                robot_pose.orientation.z, robot_pose.orientation.w,
            ])[2]
            goal = (
                robot_pose.position.x + delta_x,
                robot_pose.position.y + delta_y,
                robot_yaw,
            )
            self.task._status(
                "Camera framing adjustment: dx=%.3f m dy=%.3f m target=(%.3f, %.3f, %.3f)"
                % (delta_x, delta_y, point_camera.x, point_camera.y, point_camera.z)
            )
            if not self._position_base(goal, "camera framing"):
                return False
        return self._project_box(pose) is not None

    @staticmethod
    def _detect_central_cube(image):
        """Return the bright cube face closest to the calibrated image centre.

        Gazebo's camera plugin renders in a frame rotated from the retained
        URDF optical link, so its reported link pose cannot be used reliably
        for pixel projection.  The target face is deliberately centred before
        capture and has a bright label against the dark, uniform background.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = gray.shape
        image_centre = (width * 0.5, height * 0.5)
        candidates = []
        for contour in contours:
            left, top, box_width, box_height = cv2.boundingRect(contour)
            area = box_width * box_height
            if box_width < 12 or box_height < 12 or area < 500:
                continue
            if box_width > width * 0.9 or box_height > height * 0.9:
                continue
            centre = (left + box_width * 0.5, top + box_height * 0.5)
            distance = math.hypot(centre[0] - image_centre[0], centre[1] - image_centre[1])
            candidates.append((distance, left, top, left + box_width, top + box_height))
        if not candidates:
            return None
        _, left, top, right, bottom = min(candidates)
        return float(left), float(top), float(right), float(bottom)

    def _labels(self, target_model, image):
        box = self._detect_central_cube(image)
        if box is None:
            return [], []
        left, top, right, bottom = box
        width = float(image.shape[1])
        height = float(image.shape[0])
        centre_x = (left + right) * 0.5 / width
        centre_y = (top + bottom) * 0.5 / height
        box_width = (right - left) / width
        box_height = (bottom - top) / height
        row = "%d %.6f %.6f %.6f %.6f" % (
            CLASS_BY_MODEL[target_model], centre_x, centre_y, box_width, box_height
        )
        metadata = [{
            "model": target_model,
            "class_id": CLASS_BY_MODEL[target_model],
            "bbox_px": box,
            "source": "rgb_target_contour",
        }]
        return [row], metadata

    def _capture_frame(self, target_model, target_region, poses, frame_index):
        with self.image_lock:
            message = self.latest_image
        if message is None:
            raise rospy.ROSException("no RGB frame is available")
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            raise rospy.ROSException("could not convert camera image: %s" % error)

        label_rows, label_metadata = self._labels(target_model, image)
        if not label_rows:
            raise rospy.ROSException("could not locate the centred target cube in the RGB frame")
        filename = "%s_%s_%s_%02d" % (
            self.run_id, target_region, target_model, frame_index
        )
        image_path = os.path.join(self.images_dir, filename + ".jpg")
        label_path = os.path.join(self.labels_dir, filename + ".txt")
        if not cv2.imwrite(image_path, image):
            raise rospy.ROSException("could not write %s" % image_path)
        with open(label_path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(label_rows))
            if label_rows:
                handle.write("\n")

        record = {
            "image": os.path.relpath(image_path, self.dataset_dir),
            "label": os.path.relpath(label_path, self.dataset_dir),
            "target_model": target_model,
            "target_region": target_region,
            "sim_time": message.header.stamp.to_sec(),
            "boxes": label_metadata,
            "cube_world_poses": {
                model: {
                    "x": poses[model].position.x,
                    "y": poses[model].position.y,
                    "z": poses[model].position.z,
                }
                for model in ALL_CUBES
            },
        }
        with open(self.metadata_path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        rospy.loginfo("Saved %s (%d YOLO boxes)", image_path, len(label_rows))

    def run(self):
        self.task._wait_for_services()
        self._wait_for_camera()
        poses = self._cube_poses()
        ordered = sorted(ALL_CUBES, key=lambda model: poses[model].position.x)
        arm_started = False

        for model in ordered:
            pickup_goal, region = self._pickup_goal(model, poses)
            transport = "direct positioning" if self.direct_positioning else "navigation"
            self.task._status("Dataset capture: %s to %s (%s)" % (transport, model, region))
            if not self._position_base(pickup_goal, "%s dataset capture" % region):
                raise rospy.ROSException("cannot precisely align at %s capture pose" % region)
            if not arm_started:
                self.task._start_arm_control()
                arm_started = True
            self.task._set_gripper(1.0)
            self.task._move_arm(self.arm_capture, self.arm_capture_duration)
            if not self._centre_camera_view(poses[model]):
                raise rospy.ROSException("could not frame %s in the RGB camera" % model)
            rospy.sleep(self.settle_seconds)

            # Refresh live poses before projecting labels in case physics has moved a cube.
            poses = self._cube_poses()
            for frame_index in range(self.frames_per_cube):
                self._capture_frame(model, region, poses, frame_index)
                rospy.sleep(self.frame_interval)
            # Confirm the camera/gripper can still reach the real grasp pose
            # before parking for travel to the next cube.
            self.task._move_arm(self.task.arm_grasp, self.task.arm_grasp_duration)
            self.task._move_arm(self.park_pose, self.task.arm_grasp_duration)

        self.task._status("DATASET DONE: images and YOLO labels saved to %s" % self.dataset_dir)


if __name__ == "__main__":
    try:
        CubeDatasetCapture().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("capture_cube_dataset failed: %s", error)
        raise
