#!/usr/bin/env python3
"""Set the v2 initial arm posture once, without starting arm controllers."""

import time
import xml.etree.ElementTree as ET

import rospy
import tf.transformations as transformations
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from gazebo_msgs.srv import (
    GetLinkState,
    GetWorldProperties,
    SetLinkProperties,
    SetModelConfiguration,
)
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty


MIMIC_JOINTS = {
    "l_joint": -1.0,
    "r_in_joint": 1.0,
    "l_in_joint": -1.0,
    "r_out_joint": -1.0,
    "l_out_joint": 1.0,
}
ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
V2_INITIAL_POSE = [0.0, 1.60, -2.20, -1.00, 0.0]
ARM_LINKS = [
    "arm_link1", "arm_link2", "arm_link3", "arm_link4", "arm_link5", "arm_link6",
    "camera_mount_link", "camera_depth_link", "camera_depth_optical_frame",
    "r_link", "l_link", "r_in_link", "r_out_link", "l_in_link", "l_out_link",
    "tcp_link",
]


class InitialArmPoseSetter:
    def __init__(self):
        rospy.init_node("set_arm_initial_pose")
        self.model_name = rospy.get_param("~model_name", "car3")
        self.robot_description_param = rospy.get_param(
            "~robot_description_param", "robot_description"
        )
        self.arm_positions = self._pose_param(
            rospy.get_param("~arm_initial_positions", V2_INITIAL_POSE)
        )
        self.gripper_open = float(rospy.get_param("~gripper_open_position", 1.0))
        self.model_wait_timeout = float(rospy.get_param("~model_wait_timeout", 15.0))
        self.pause_physics = bool(rospy.get_param("~pause_physics", True))
        # Directly setting a highly folded pose while physics is paused makes
        # the CAD meshes appear to snap apart.  By default use a controller
        # trajectory during preparation instead; this flag remains available
        # only for recovery/debug use.
        self.direct_initial_configuration = bool(
            rospy.get_param("~direct_initial_configuration", False)
        )
        # Holding the arm links gravity-free is a Gazebo physics setting, not
        # a controller command.  It prevents the unpowered v3 arm from
        # falling back to its vertical pose while the base is navigating.
        self.disable_arm_gravity = bool(rospy.get_param("~disable_arm_gravity", True))
        self.arm_links = rospy.get_param("~arm_links", ARM_LINKS)
        self.ready_settle_time = float(rospy.get_param("~ready_settle_time", 0.6))
        # In v3 gazebo_ros_control resets unclaimed position interfaces to
        # zero.  Reapply the pose through Gazebo (not through a controller)
        # while driving, then stop before arm_controller is enabled.
        self.hold_initial_pose = bool(rospy.get_param("~hold_initial_pose", False))
        self.hold_rate = float(rospy.get_param("~hold_rate", 30.0))
        self._hold_timer = None
        self.controller_duration = float(rospy.get_param("~controller_duration", 4.0))
        self.transition_steps = max(1, int(rospy.get_param("~transition_steps", 2)))
        self.controller_command_repeats = int(
            rospy.get_param("~controller_command_repeats", 8)
        )
        self.link_inertials = self._read_link_inertials()

        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/set_model_configuration")
        rospy.wait_for_service("/gazebo/get_link_state")
        rospy.wait_for_service("/controller_manager/switch_controller")
        self.get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.set_configuration = rospy.ServiceProxy(
            "/gazebo/set_model_configuration", SetModelConfiguration
        )
        self.get_link_state = rospy.ServiceProxy(
            "/gazebo/get_link_state", GetLinkState
        )
        self.set_link_properties = rospy.ServiceProxy(
            "/gazebo/set_link_properties", SetLinkProperties
        )
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.switch_controller = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )
        self.arm_pub = rospy.Publisher(
            "/arm_controller/command", JointTrajectory, queue_size=1
        )
        self.gripper_pub = rospy.Publisher(
            "/gripper_controller/command", Float64, queue_size=1
        )
        self.ready_pub = rospy.Publisher(
            "/sim_task3/arm_initial_pose_ready", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/sim_task3/arm_control_enabled", Bool, self._arm_control_enabled_cb,
            queue_size=1,
        )

    @staticmethod
    def _pose_param(values):
        # roslaunch <param value="a,b,c,d,e"> arrives as a string, whereas
        # a rosparam YAML list arrives as a list.  Support both so the
        # preparation launch can expose the calibrated initial posture.
        if isinstance(values, str):
            values = [value.strip() for value in values.split(",") if value.strip()]
        if not isinstance(values, (list, tuple)) or len(values) != len(ARM_JOINTS):
            raise rospy.ROSException("arm_initial_positions must contain five angles")
        return [float(value) for value in values]

    def _model_is_ready(self):
        try:
            return self.model_name in self.get_world().model_names
        except rospy.ServiceException:
            return False

    def _robot_links_are_ready(self):
        if not self._model_is_ready():
            return False
        try:
            return self.get_link_state(
                "%s::tcp_link" % self.model_name, "world"
            ).success
        except rospy.ServiceException:
            return False

    @staticmethod
    def _float_values(text, count):
        values = [float(value) for value in text.split()]
        if len(values) != count:
            raise ValueError("expected %d values, got %s" % (count, text))
        return values

    def _read_link_inertials(self):
        try:
            robot = ET.fromstring(rospy.get_param(self.robot_description_param))
        except (ET.ParseError, KeyError) as error:
            raise rospy.ROSException("cannot read robot inertials: %s" % error)

        values = {}
        for link in robot.findall("link"):
            name = link.get("name")
            if name not in self.arm_links:
                continue
            inertial = link.find("inertial")
            if inertial is None:
                continue
            origin = inertial.find("origin")
            xyz = self._float_values(origin.get("xyz", "0 0 0"), 3)
            rpy = self._float_values(origin.get("rpy", "0 0 0"), 3)
            q = transformations.quaternion_from_euler(*rpy)
            com = Pose(
                position=Point(*xyz),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            )
            inertia = inertial.find("inertia")
            values[name] = (
                com,
                float(inertial.find("mass").get("value")),
                float(inertia.get("ixx")), float(inertia.get("ixy")),
                float(inertia.get("ixz")), float(inertia.get("iyy")),
                float(inertia.get("iyz")), float(inertia.get("izz")),
            )
        return values

    def _set_arm_gravity(self, enabled):
        for link in self.arm_links:
            scoped_name = "%s::%s" % (self.model_name, link)
            inertial = self.link_inertials.get(link)
            if inertial is None:
                rospy.logwarn("no inertial data for %s", scoped_name)
                continue
            try:
                result = self.set_link_properties(
                    scoped_name,
                    inertial[0],
                    enabled,
                    *inertial[1:]
                )
                if not result.success:
                    rospy.logwarn("cannot set gravity on %s: %s", scoped_name, result.status_message)
            except rospy.ServiceException as error:
                rospy.logwarn("gravity setup failed for %s: %s", scoped_name, error)

    def _set_initial_configuration(self, warn=True):
        names = list(ARM_JOINTS) + ["r_joint"] + list(MIMIC_JOINTS.keys())
        positions = list(self.arm_positions) + [self.gripper_open]
        positions.extend(self.gripper_open * ratio for ratio in MIMIC_JOINTS.values())
        try:
            result = self.set_configuration(
                self.model_name, self.robot_description_param, names, positions
            )
            if not result.success and warn:
                rospy.logwarn("could not set initial arm pose: %s", result.status_message)
            return result.success
        except rospy.ServiceException as error:
            if warn:
                rospy.logwarn("initial arm pose service failed: %s", error)
            return False

    def _hold_initial_pose_cb(self, _event):
        self._set_initial_configuration(warn=False)

    def _arm_control_enabled_cb(self, message):
        if not message.data:
            return
        if self._hold_timer is not None:
            self._hold_timer.shutdown()
            self._hold_timer = None
        rospy.loginfo("Arm controller is taking over; stopped Gazebo initial-pose hold")
        rospy.signal_shutdown("arm controller enabled")

    def _switch_controllers(self, start, stop):
        request = SwitchControllerRequest()
        request.start_controllers = list(start)
        request.stop_controllers = list(stop)
        request.strictness = SwitchControllerRequest.STRICT
        request.start_asap = True
        request.timeout = 2.0
        try:
            return self.switch_controller(request).ok
        except rospy.ServiceException as error:
            rospy.logwarn("arm controller switch failed: %s", error)
            return False

    def _set_pose_then_stop_controllers(self):
        # v3 resets a passive PositionJointInterface to zero.  Set the pose
        # while the robot is still parked, then stop both controllers before
        # task navigation starts.  This is the only arm-controller use before
        # pickup; no arm/gripper controller runs during driving.
        started = False
        for _ in range(30):
            if self._switch_controllers(["arm_controller", "gripper_controller"], []):
                started = True
                break
            rospy.sleep(0.1)
        if not started:
            rospy.logwarn("could not start controllers for the initial arm pose")
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)
        # Move in evenly spaced stages from the spawn posture.  Stopping at
        # the intermediate point avoids a sharp simultaneous j3/j4 rotation.
        for step in range(1, self.transition_steps + 1):
            fraction = float(step) / self.transition_steps
            point = JointTrajectoryPoint()
            point.positions = [fraction * value for value in self.arm_positions]
            point.velocities = [0.0] * len(ARM_JOINTS)
            point.time_from_start = rospy.Duration(self.controller_duration * fraction)
            trajectory.points.append(point)
        for _ in range(max(1, self.controller_command_repeats)):
            self.arm_pub.publish(trajectory)
            self.gripper_pub.publish(Float64(data=self.gripper_open))
            rospy.sleep(0.05)
        rospy.sleep(self.controller_duration + 0.2)

        stopped = self._switch_controllers([], ["arm_controller", "gripper_controller"])
        if stopped:
            rospy.loginfo(
                "calibrated initial arm pose applied smoothly in %d stages; "
                "arm/gripper controllers stopped", self.transition_steps
            )
        else:
            rospy.logwarn("initial pose applied but controllers did not stop cleanly")
        return stopped

    def run(self):
        deadline = rospy.Time.now() + rospy.Duration(self.model_wait_timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self._robot_links_are_ready():
                break
            rate.sleep()
        else:
            rospy.logwarn("%s did not spawn; skipped initial arm pose", self.model_name)
            return

        # Gazebo's ros_control model plugin finishes initialising shortly after
        # the model appears.  Configuring before then is overwritten by its
        # zero-joint startup state, which is why the arm looked vertical.
        time.sleep(self.ready_settle_time)

        paused = False
        try:
            if self.pause_physics:
                self.pause()
                paused = True
            if self.direct_initial_configuration and not self._set_initial_configuration():
                return
            if self.disable_arm_gravity:
                self._set_arm_gravity(False)
                if self.direct_initial_configuration:
                    rospy.loginfo("Set calibrated pose directly and disabled arm gravity: %s", self.arm_positions)
                else:
                    rospy.loginfo(
                        "Disabled arm gravity; moving smoothly to calibrated pose: %s",
                        self.arm_positions,
                    )
            else:
                rospy.loginfo("Moving smoothly to calibrated pose: %s", self.arm_positions)
        except rospy.ServiceException as error:
            rospy.logwarn("initial arm pose service failed: %s", error)
        finally:
            if paused:
                try:
                    self.unpause()
                except rospy.ServiceException:
                    pass

        self._set_pose_then_stop_controllers()
        self.ready_pub.publish(Bool(data=True))
        if self.hold_initial_pose:
            self._hold_timer = rospy.Timer(
                rospy.Duration(1.0 / max(1.0, self.hold_rate)),
                self._hold_initial_pose_cb,
            )
            rospy.loginfo("Holding v2 initial pose through Gazebo at %.1f Hz", self.hold_rate)
            rospy.spin()


if __name__ == "__main__":
    InitialArmPoseSetter().run()
