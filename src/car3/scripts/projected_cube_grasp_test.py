#!/usr/bin/env python3
"""One real grasp trial using the user's calibrated arm poses and XY projection."""

import math
import time

import rospy
import tf.transformations as transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetLinkState, GetModelState, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
DEFAULT_PRE = [0.0001, 1.1000, 0.7000, 1.2000, 0.0]
DEFAULT_GRASP = [0.0001, 1.3001, 0.5000, 1.0000, 0.0]
SAVED_YAW = -3.1339
SAVED_XY = (-1.6327, -0.3519)


def yaw_quaternion(yaw):
    q = transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class ProjectedCubeGraspTest:
    def __init__(self):
        rospy.init_node("projected_cube_grasp_test")
        # spawn_cubes.py deliberately randomises model names across the three
        # areas.  The first/manual calibration refers to the *left* area, not
        # to a fixed cube_0/1/2 model name.
        self.cube_model = rospy.get_param("~cube_model", "")
        self.target_region = rospy.get_param("~target_region", "left")
        self.pre_pose = self._pose_param("~pre_pose", DEFAULT_PRE)
        self.grasp_pose = self._pose_param("~grasp_pose", DEFAULT_GRASP)
        self.saved_x = float(rospy.get_param("~saved_x", SAVED_XY[0]))
        self.saved_y = float(rospy.get_param("~saved_y", SAVED_XY[1]))
        self.saved_yaw = float(rospy.get_param("~saved_yaw", SAVED_YAW))
        self.motion_seconds = float(rospy.get_param("~motion_seconds", 1.4))
        self.success_distance = float(rospy.get_param("~success_distance", 0.025))

        self.get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", Float64, queue_size=1)
        self.result_pub = rospy.Publisher(
            "/sim_task3/projected_grasp_result", String, queue_size=1, latch=True
        )

    @staticmethod
    def _pose_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            value = [item.strip() for item in value.split(",") if item.strip()]
        if not isinstance(value, (list, tuple)) or len(value) != 5:
            raise rospy.ROSException("%s must contain five joint angles" % name)
        return [float(item) for item in value]

    def _select_target_cube(self):
        if self.cube_model:
            response = self.get_model(self.cube_model, "world")
            return self.cube_model, response
        candidates = []
        for name in ("cube_0", "cube_1", "cube_2"):
            response = self.get_model(name, "world")
            if response.success:
                candidates.append((name, response))
        if not candidates:
            return "", None
        if self.target_region == "left":
            return min(candidates, key=lambda item: item[1].pose.position.x)
        if self.target_region == "right":
            return max(candidates, key=lambda item: item[1].pose.position.x)
        if self.target_region == "top":
            return max(candidates, key=lambda item: item[1].pose.position.y)
        raise rospy.ROSException("unknown target_region: %s" % self.target_region)

    def _publish_arm(self, pose):
        message = JointTrajectory()
        message.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(pose)
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = rospy.Duration(self.motion_seconds)
        message.points = [point]
        for _ in range(3):
            self.arm_pub.publish(message)
            time.sleep(0.05)
        # Wall-clock wait keeps this script robust even if /clock is slow.
        time.sleep(max(1.0, self.motion_seconds * 1.5))

    def _set_car_pose(self, x, y):
        state = ModelState()
        state.model_name = "car3"
        state.pose = Pose(
            position=Point(x=x, y=y, z=0.01),
            orientation=yaw_quaternion(self.saved_yaw),
        )
        state.twist = Twist()
        state.reference_frame = "world"
        response = self.set_model(state)
        if not response.success:
            raise rospy.ROSException("failed to set car pose: %s" % response.status_message)

    @staticmethod
    def _distance(first, second):
        return math.sqrt(
            (first.x - second.x) ** 2 +
            (first.y - second.y) ** 2 +
            (first.z - second.z) ** 2
        )

    def run(self):
        for service in ("/gazebo/get_model_state", "/gazebo/get_link_state", "/gazebo/set_model_state"):
            rospy.wait_for_service(service)
        deadline = time.monotonic() + 15.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.cube_model, cube = self._select_target_cube()
            if cube is not None and cube.success:
                break
            time.sleep(0.1)
        else:
            raise rospy.ROSException("no target cube was spawned")

        # The object stays where the official random spawner placed it.
        self._set_car_pose(self.saved_x, self.saved_y)
        time.sleep(0.3)
        self.gripper_pub.publish(Float64(data=1.0))
        time.sleep(0.35)
        self._publish_arm(self.pre_pose)
        self._publish_arm(self.grasp_pose)

        cube = self.get_model(self.cube_model, "world")
        tcp = self.get_link("car3::tcp_link", "world")
        if not cube.success or not tcp.success:
            raise rospy.ROSException("could not read cube or TCP pose")

        # Adjust only base X/Y, keeping saved heading and the calibrated arm pose.
        delta_x = cube.pose.position.x - tcp.link_state.pose.position.x
        delta_y = cube.pose.position.y - tcp.link_state.pose.position.y
        self._set_car_pose(self.saved_x + delta_x, self.saved_y + delta_y)
        time.sleep(0.35)

        tcp = self.get_link("car3::tcp_link", "world")
        cube = self.get_model(self.cube_model, "world")
        before = self._distance(tcp.link_state.pose.position, cube.pose.position)
        dz = cube.pose.position.z - tcp.link_state.pose.position.z
        self.gripper_pub.publish(Float64(data=0.76))
        time.sleep(1.0)
        tcp = self.get_link("car3::tcp_link", "world")
        cube = self.get_model(self.cube_model, "world")
        after = self._distance(tcp.link_state.pose.position, cube.pose.position)
        success = after <= self.success_distance
        result = (
            "{status}; cube={cube}; before={before:.4f}m; after={after:.4f}m; "
            "vertical_delta_before={dz:.4f}m"
        ).format(
            status="SUCCESS" if success else "NOT_ATTACHED",
            cube=self.cube_model, before=before, after=after, dz=dz,
        )
        rospy.loginfo(result)
        self.result_pub.publish(String(data=result))


if __name__ == "__main__":
    try:
        ProjectedCubeGraspTest().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("projected cube grasp test failed: %s", error)
        raise
