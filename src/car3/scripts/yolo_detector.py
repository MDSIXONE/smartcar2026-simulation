#!/usr/bin/env python3
"""Publish YOLOv3-tiny detections for the Gazebo RGB camera.

The model process is started once and uses the local ``yolo_pipe`` NDJSON
protocol.  Frames are intentionally throttled so Gazebo navigation and camera
simulation remain responsive while the CPU detector is active.
"""

import json
import os
import subprocess
import tempfile
import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector:
    def __init__(self):
        rospy.init_node("yolo_detector")

        self.darknet_dir = os.path.expanduser(
            rospy.get_param("~darknet_dir", "~/smartcar2026-models/darknet-yolov3")
        )
        self.binary = self._path_param("~binary", "yolo_pipe")
        self.config = self._path_param("~config", "cfg/yolov3-tiny-3cls.cfg")
        self.weights = self._path_param(
            "~weights", "models/yolov3-tiny-3cls_best.weights"
        )
        self.names = self._path_param("~names", "data/obj.names")
        self.threshold = float(rospy.get_param("~threshold", 0.25))
        self.nms = float(rospy.get_param("~nms", 0.45))
        self.start_timeout = float(rospy.get_param("~start_timeout", 120.0))
        self.inference_rate = max(0.1, float(rospy.get_param("~inference_rate", 2.0)))
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/sim_task3/yolo/detections"
        )
        self.status_topic = rospy.get_param("~status_topic", "/sim_task3/yolo/status")

        for path in (self.binary, self.config, self.weights, self.names):
            if not os.path.isfile(path):
                raise rospy.ROSException("YOLO runtime file is missing: %s" % path)
        if not os.access(self.binary, os.X_OK):
            raise rospy.ROSException("YOLO binary is not executable: %s" % self.binary)

        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.latest_image = None
        self.process = None
        self.stderr_thread = None
        self.pipe_ready = threading.Event()
        self.runtime_dir = tempfile.TemporaryDirectory(prefix="car3-yolo-")
        self.frame_path = os.path.join(self.runtime_dir.name, "frame.jpg")

        self.detections_pub = rospy.Publisher(
            self.detections_topic, String, queue_size=10
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.camera_topic, Image, self._image_callback, queue_size=1)
        rospy.on_shutdown(self._shutdown)

        self._start_pipe()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.inference_rate), self._detect_latest_frame
        )
        self._publish_status("ready", rate_hz=self.inference_rate)

    def _path_param(self, name, default_relative_path):
        value = os.path.expanduser(rospy.get_param(name, default_relative_path))
        if not os.path.isabs(value):
            value = os.path.join(self.darknet_dir, value)
        return os.path.abspath(value)

    def _publish_status(self, state, **details):
        payload = {"state": state}
        payload.update(details)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _drain_stderr(self):
        for line in iter(self.process.stderr.readline, ""):
            text = line.rstrip()
            if "###READY###" in text:
                self.pipe_ready.set()
            elif text:
                rospy.logdebug("yolo_pipe: %s", text)
        self.pipe_ready.set()

    def _start_pipe(self):
        command = [
            self.binary,
            self.config,
            self.weights,
            self.names,
            str(self.threshold),
            str(self.nms),
        ]
        rospy.loginfo("Starting persistent YOLOv3 detector: %s", " ".join(command))
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            close_fds=True,
        )
        self.stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
        self.stderr_thread.start()
        if not self.pipe_ready.wait(self.start_timeout):
            raise rospy.ROSException("Timed out while loading YOLOv3 weights")
        if self.process.poll() is not None:
            raise rospy.ROSException("yolo_pipe exited while loading the model")

        line = self.process.stdout.readline()
        try:
            ready = json.loads(line)
        except (TypeError, ValueError) as error:
            raise rospy.ROSException("invalid yolo_pipe ready response: %r" % line) from error
        if ready.get("status") != "ready":
            raise rospy.ROSException("yolo_pipe failed to load model: %s" % ready)
        rospy.loginfo("YOLOv3 ready: %s", ready)

    def _image_callback(self, message):
        with self.image_lock:
            self.latest_image = message

    def _detect_latest_frame(self, _event):
        if not self.processing_lock.acquire(blocking=False):
            return
        try:
            with self.image_lock:
                message = self.latest_image
            if message is None:
                return
            if self.process is None or self.process.poll() is not None:
                raise rospy.ROSException("yolo_pipe is no longer running")

            try:
                image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            except CvBridgeError as error:
                rospy.logwarn_throttle(5.0, "cannot convert RGB frame: %s", error)
                return
            if not cv2.imwrite(self.frame_path, image):
                raise rospy.ROSException("cannot write temporary YOLO input frame")

            self.process.stdin.write(self.frame_path + "\n")
            self.process.stdin.flush()
            response_line = self.process.stdout.readline()
            if not response_line:
                raise rospy.ROSException("yolo_pipe closed its output stream")
            response = json.loads(response_line)
            if "error" in response:
                rospy.logwarn_throttle(5.0, "YOLO input error: %s", response["error"])
                return

            payload = {
                "stamp": {"secs": message.header.stamp.secs, "nsecs": message.header.stamp.nsecs},
                "frame_id": message.header.frame_id,
                "width": response.get("width", message.width),
                "height": response.get("height", message.height),
                "detections": response.get("detections", []),
            }
            self.detections_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            rospy.loginfo_throttle(
                2.0, "YOLOv3: %d detection(s)", len(payload["detections"])
            )
        except (OSError, ValueError, rospy.ROSException) as error:
            rospy.logerr_throttle(5.0, "YOLOv3 inference failed: %s", error)
            self._publish_status("error", message=str(error))
        finally:
            self.processing_lock.release()

    def _shutdown(self):
        if hasattr(self, "timer"):
            self.timer.shutdown()
        if self.process is not None and self.process.poll() is None:
            try:
                self.process.stdin.write("quit\n")
                self.process.stdin.flush()
                self.process.wait(timeout=5.0)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                self.process.terminate()
        if hasattr(self, "runtime_dir"):
            self.runtime_dir.cleanup()


if __name__ == "__main__":
    try:
        YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("yolo_detector failed: %s", error)
        raise
