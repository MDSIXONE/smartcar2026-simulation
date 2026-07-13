#!/usr/bin/env python3
"""Safe terminal keyboard teleoperation for the car base only."""

import select
import sys
import termios
import time
import tty

import rospy
import tf.transformations as transformations
import yaml
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class KeyboardDrive:
    def __init__(self):
        rospy.init_node("keyboard_drive")
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.18))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.55))
        self.deadman_timeout = float(rospy.get_param("~deadman_timeout", 0.35))
        self.model_name = rospy.get_param("~model_name", "car3")
        self.pose_save_path = rospy.get_param(
            "~pose_save_path", "/home/car/.ros/car3_saved_pose.yaml"
        )
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pose_pub = rospy.Publisher(
            "/sim_task3/saved_car_pose", String, queue_size=1, latch=True
        )
        self.get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        self.settings = termios.tcgetattr(sys.stdin)

    @staticmethod
    def _help():
        print("\nKeyboard car control (this terminal must stay focused)")
        print("  w / i : forward       s / , : reverse")
        print("  a / j : turn left     d / l : turn right")
        print("  J     : strafe left   L     : strafe right")
        print("  p     : save current x, y, yaw")
        print("  x / k : stop          + / - : change speed")
        print("  q     : quit")
        print("  Hold a movement key; releasing it stops the car within 0.35 s.\n")

    @staticmethod
    def _read_key(timeout):
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if readable:
            return sys.stdin.read(1)
        return ""

    def _publish(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.publisher.publish(command)

    def _save_pose(self):
        try:
            result = self.get_model_state(self.model_name, "world")
        except rospy.ServiceException as error:
            print("\rCould not read vehicle pose: %s" % error)
            return
        if not result.success:
            print("\rCould not read vehicle pose: %s" % result.status_message)
            return
        position = result.pose.position
        orientation = result.pose.orientation
        yaw = transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])[2]
        pose = {
            "frame": "map",
            "x": round(position.x, 4),
            "y": round(position.y, 4),
            "yaw": round(yaw, 4),
        }
        try:
            with open(self.pose_save_path, "w", encoding="utf-8") as output:
                yaml.safe_dump(pose, output, allow_unicode=True, sort_keys=False)
        except OSError as error:
            print("\rPose read, but could not save file: %s" % error)
            return
        text = "frame=map | x={x:.4f} m | y={y:.4f} m | yaw={yaw:.4f} rad".format(**pose)
        self.pose_pub.publish(String(data=text))
        print("\rSAVED_CAR_POSE: " + text)
        print("Saved to: " + self.pose_save_path)

    def run(self):
        self._help()
        last_motion = 0.0
        try:
            tty.setraw(sys.stdin.fileno())
            while not rospy.is_shutdown():
                key = self._read_key(0.05)
                now = time.monotonic()
                if key in ("q", "\x03"):
                    return
                if key in ("w", "i"):
                    self._publish(self.linear_speed, 0.0)
                    last_motion = now
                elif key in ("s", ","):
                    self._publish(-self.linear_speed, 0.0)
                    last_motion = now
                elif key in ("a", "j"):
                    self._publish(0.0, self.angular_speed)
                    last_motion = now
                elif key in ("d", "l"):
                    self._publish(0.0, -self.angular_speed)
                    last_motion = now
                elif key == "J":
                    # ROS base_link convention: +Y points to the left.
                    command = Twist()
                    command.linear.y = self.linear_speed
                    self.publisher.publish(command)
                    last_motion = now
                elif key == "L":
                    command = Twist()
                    command.linear.y = -self.linear_speed
                    self.publisher.publish(command)
                    last_motion = now
                elif key in ("x", "k", " "):
                    self._publish()
                    last_motion = 0.0
                elif key == "p":
                    self._publish()
                    last_motion = 0.0
                    self._save_pose()
                elif key == "+":
                    self.linear_speed *= 1.15
                    self.angular_speed *= 1.15
                    print("\rlinear=%.2f angular=%.2f    " % (self.linear_speed, self.angular_speed), end="")
                elif key == "-":
                    self.linear_speed *= 0.85
                    self.angular_speed *= 0.85
                    print("\rlinear=%.2f angular=%.2f    " % (self.linear_speed, self.angular_speed), end="")
                elif last_motion and now - last_motion > self.deadman_timeout:
                    self._publish()
                    last_motion = 0.0
        finally:
            self._publish()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    KeyboardDrive().run()
