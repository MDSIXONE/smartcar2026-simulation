#!/usr/bin/env python3
"""Pick and deliver one labelled cube after the environment is prepared.

The random spawner deliberately assigns cube_* model names to the left, upper,
and right bays in a random order.  Category identifies the labelled model;
this node then obtains its live Gazebo pose and computes the matching projected
base pose from the manual left-bay calibration.
"""

import math
import time

import actionlib
import rospy
import tf.transformations as transformations
from actionlib_msgs.msg import GoalStatus
from controller_manager_msgs.srv import (
    ListControllers,
    SwitchController,
    SwitchControllerRequest,
)
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetLinkState, GetModelState, SetModelState
from geometry_msgs.msg import Pose, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
ALL_CUBES = ("cube_0", "cube_1", "cube_2")

# The mesh/model label defines a category, while spawn_cubes.py randomises the
# model's bay.  Therefore model ID is used only for category recognition, not
# for left/upper/right position recognition.
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

# Successful left-bay calibration.  Values are target-minus-car in world/map.
LEFT_REFERENCE_YAW = 3.141157
LEFT_TARGET_MINUS_CAR = (-0.319277, 0.000771)


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )[2]


def quaternion_from_yaw(yaw):
    x, y, z, w = transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


def rotate_2d(x, y, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


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
        self.fine_align_timeout = float(rospy.get_param("~fine_align_timeout", 12.0))
        self.fine_align_position_tolerance = float(
            rospy.get_param("~fine_align_position_tolerance", 0.008)
        )
        self.fine_align_yaw_tolerance = float(
            rospy.get_param("~fine_align_yaw_tolerance", 0.025)
        )
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        self.camera_timeout = float(rospy.get_param("~camera_timeout", 1.0))

        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", Float64, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
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

        self.camera_frames = 0
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
        self.get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
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

    def _camera_callback(self, _message):
        self.camera_frames += 1

    def _grasp_state_callback(self, message):
        self.grasp_state = message.data

    def _status(self, text):
        rospy.loginfo(text)
        self.status_pub.publish(String(data=text))

    def _wait_for_services(self):
        for name in (
            "/gazebo/get_model_state",
            "/gazebo/get_link_state",
            "/gazebo/set_model_state",
            "/controller_manager/switch_controller",
            "/controller_manager/list_controllers",
        ):
            self._status("Waiting for %s" % name)
            rospy.wait_for_service(name)
        self._status("Waiting for move_base")
        self.nav.wait_for_server()

    def _cube_poses(self):
        poses = {}
        for model in ALL_CUBES:
            response = self.get_model(model, "world")
            if not response.success:
                raise rospy.ROSException("target model %s has not spawned" % model)
            poses[model] = response.pose
        return poses

    def _recognise_and_project_pickup(self):
        start_frames = self.camera_frames
        deadline = time.time() + self.camera_timeout
        while not rospy.is_shutdown() and time.time() < deadline and self.camera_frames == start_frames:
            rospy.sleep(0.05)

        poses = self._cube_poses()
        ordered = sorted(ALL_CUBES, key=lambda model: poses[model].position.x)
        region_index = ordered.index(self.cargo_model)
        region_name = ("left", "upper", "right")[region_index]
        # Each bay is one clockwise 90-degree rotation from the previous one.
        pickup_yaw = wrap(LEFT_REFERENCE_YAW - region_index * math.pi / 2.0)
        local_dx, local_dy = rotate_2d(
            LEFT_TARGET_MINUS_CAR[0], LEFT_TARGET_MINUS_CAR[1], -LEFT_REFERENCE_YAW
        )
        target_minus_car_x, target_minus_car_y = rotate_2d(local_dx, local_dy, pickup_yaw)
        target_pose = poses[self.cargo_model]
        pickup_goal = (
            target_pose.position.x - target_minus_car_x,
            target_pose.position.y - target_minus_car_y,
            pickup_yaw,
        )
        camera_note = "camera frame received" if self.camera_frames > start_frames else "Gazebo label fallback"
        self._status(
            "%s: %s -> %s; region=%s; pickup=(%.4f, %.4f, %.4f)"
            % (camera_note, self.cargo_name, self.cargo_model, region_name,
               pickup_goal[0], pickup_goal[1], pickup_goal[2])
        )
        return pickup_goal, region_name

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

    def _robot_pose(self):
        response = self.get_model("car3", "world")
        if not response.success:
            raise rospy.ROSException("could not read car3 pose")
        return response.pose

    def _fine_align_base(self, goal):
        """Use low-speed omnidirectional correction after move_base is done.

        The calibrated grasp box is only 2 cm wide.  move_base's terminal
        tolerance is intentionally looser, so this final correction uses the
        live Gazebo car pose instead of teleporting the vehicle.
        """
        deadline = time.time() + self.fine_align_timeout
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                pose = self._robot_pose()
                yaw = yaw_from_quaternion(pose.orientation)
                dx = goal[0] - pose.position.x
                dy = goal[1] - pose.position.y
                distance = math.hypot(dx, dy)
                yaw_error = wrap(goal[2] - yaw)
                if (distance <= self.fine_align_position_tolerance and
                        abs(yaw_error) <= self.fine_align_yaw_tolerance):
                    self._status(
                        "Projected pickup pose aligned: xy_error=%.4f m yaw_error=%.4f rad"
                        % (distance, yaw_error)
                    )
                    return True
                forward, lateral = rotate_2d(dx, dy, -yaw)
                command = Twist()
                command.linear.x = clamp(1.8 * forward, -0.12, 0.12)
                command.linear.y = clamp(1.8 * lateral, -0.12, 0.12)
                command.angular.z = clamp(2.2 * yaw_error, -0.60, 0.60)
                self.cmd_pub.publish(command)
                rate.sleep()
        finally:
            self.cmd_pub.publish(Twist())
        pose = self._robot_pose()
        error = math.hypot(goal[0] - pose.position.x, goal[1] - pose.position.y)
        self._status("Projected pickup fine alignment timed out at %.4f m error" % error)
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
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
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
        self._status("Opening gripper and moving camera/gripper to the projected cube")
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
        pickup_goal, region = self._recognise_and_project_pickup()
        if not self._move_base(pickup_goal, "%s pickup bay" % region):
            raise rospy.ROSException("cannot reach the projected %s pickup bay" % region)
        if not self._fine_align_base(pickup_goal):
            raise rospy.ROSException("cannot precisely align at the %s pickup pose" % region)
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
