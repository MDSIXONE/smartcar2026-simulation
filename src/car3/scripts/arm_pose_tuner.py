#!/usr/bin/env python3
"""Interactive five-joint arm tuner; it never starts navigation or grasping."""

import sys

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    SwitchController,
    SwitchControllerRequest,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
JOINT_MIN = -3.14
JOINT_MAX = 3.14
V2_INITIAL_POSE = [0.0, 1.60, -2.20, -1.00, 0.0]


class ArmPoseTuner:
    def __init__(self):
        rospy.init_node("arm_pose_tuner")
        self.duration = float(rospy.get_param("~duration", 1.0))
        self.target = list(V2_INITIAL_POSE)
        self.joint_positions = {}

        self.arm_pub = rospy.Publisher(
            "/arm_controller/command", JointTrajectory, queue_size=1
        )
        self.gripper_pub = rospy.Publisher(
            "/gripper_controller/command", Float64, queue_size=1
        )
        self.pose_pub = rospy.Publisher(
            "/sim_task3/calibrated_arm_pose", String, queue_size=1, latch=True
        )
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self._joint_cb)
        self.switch_controller = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )
        self.list_controllers = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers
        )

    def _joint_cb(self, message):
        self.joint_positions = {
            name: position for name, position in zip(message.name, message.position)
        }

    def _ensure_controllers(self):
        rospy.wait_for_service("/controller_manager/switch_controller")
        rospy.wait_for_service("/controller_manager/list_controllers")
        states = {
            controller.name: controller.state
            for controller in self.list_controllers().controller
        }
        needed = [
            name for name in ("arm_controller", "gripper_controller")
            if states.get(name) != "running"
        ]
        if not needed:
            return
        request = SwitchControllerRequest()
        request.start_controllers = needed
        request.strictness = SwitchControllerRequest.STRICT
        request.start_asap = True
        request.timeout = 3.0
        response = self.switch_controller(request)
        if not response.ok:
            raise rospy.ROSException("could not start arm/gripper controllers")

    def run(self):
        self._ensure_controllers()
        print("\nArm pose tuner ready. The automatic task is stopped.")
        print("  p a1 a2 a3 a4 a5 : set a complete pose")
        print("  jN +/-step       : alter one joint, e.g. j3 +0.05")
        print("  cur              : load the measured current pose")
        print("  home             : command the v2 initial pose")
        print("  show | save      : show target and measured pose / publish target")
        print("  open | close     : control gripper")
        print("  t seconds        : set motion duration")
        print("  q                : exit\n")
        self._show()

        while not rospy.is_shutdown():
            try:
                line = input("arm> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("")
                return
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                return
            self._handle(line)

    def _handle(self, line):
        parts = line.split()
        command = parts[0].lower()
        if command in ("show", "save"):
            self._show(publish=(command == "save"))
        elif command == "home":
            self.target = list(V2_INITIAL_POSE)
            self._publish()
            self._show()
        elif command == "cur":
            if not self._load_current():
                print("No complete /joint_states received yet.")
            self._show()
        elif command == "open":
            self.gripper_pub.publish(Float64(data=1.0))
            print("gripper open")
        elif command == "close":
            self.gripper_pub.publish(Float64(data=0.76))
            print("gripper close")
        elif command == "t" and len(parts) == 2:
            self.duration = float(parts[1])
            print("duration = %.2f s" % self.duration)
        elif command == "p" and len(parts) == 6:
            candidate = [float(value) for value in parts[1:]]
            if not self._valid_pose(candidate):
                return
            self.target = candidate
            self._publish()
            self._show()
        elif command.startswith("j") and command[1:].isdigit() and len(parts) == 2:
            index = int(command[1:]) - 1
            if not 0 <= index < len(ARM_JOINTS):
                print("joint index must be 1..5")
                return
            candidate = list(self.target)
            candidate[index] += float(parts[1])
            if not self._valid_pose(candidate):
                return
            self.target = candidate
            self._publish()
            self._show()
        else:
            print("Use p, j1..j5, cur, show, save, open, close, t, or q.")

    def _load_current(self):
        if not all(name in self.joint_positions for name in ARM_JOINTS):
            return False
        self.target = [self.joint_positions[name] for name in ARM_JOINTS]
        return True

    @staticmethod
    def _valid_pose(pose):
        for index, value in enumerate(pose, start=1):
            if not JOINT_MIN <= value <= JOINT_MAX:
                print(
                    "Rejected: arm_joint%d=%.4f exceeds [%.2f, %.2f] rad"
                    % (index, value, JOINT_MIN, JOINT_MAX)
                )
                return False
        return True

    def _publish(self):
        message = JointTrajectory()
        message.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(self.target)
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = rospy.Duration(self.duration)
        message.points = [point]
        for _ in range(3):
            self.arm_pub.publish(message)
            rospy.sleep(0.05)

    def _show(self, publish=False):
        pose = ",".join("%.4f" % value for value in self.target)
        line = "[%s]" % pose
        print("TARGET_POSE = %s" % line)
        if all(name in self.joint_positions for name in ARM_JOINTS):
            actual = "[" + ",".join(
                "%.4f" % self.joint_positions[name] for name in ARM_JOINTS
            ) + "]"
            print("MEASURED_POSE = %s" % actual)
        print('task launch override: arm_pre_grasp_pose:="%s"' % pose)
        if publish:
            self.pose_pub.publish(String(data=line))
            print("CALIBRATED_INITIAL_POSE = %s" % line)
            print("published to /sim_task3/calibrated_arm_pose")


if __name__ == "__main__":
    try:
        ArmPoseTuner().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        print("error: %s" % error, file=sys.stderr)
        sys.exit(1)
