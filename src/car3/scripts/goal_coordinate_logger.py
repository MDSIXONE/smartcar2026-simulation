#!/usr/bin/env python3

import math

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped


def goal_callback(message):
    quaternion = message.pose.orientation
    yaw = tf.transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )[2]
    print(
        "RViz GOAL | frame={} | x={:.3f} m | y={:.3f} m | yaw={:.1f} deg ({:.3f} rad)".format(
            message.header.frame_id,
            message.pose.position.x,
            message.pose.position.y,
            math.degrees(yaw),
            yaw,
        ),
        flush=True,
    )
    rospy.logwarn(
        "RViz GOAL | frame=%s | x=%.3f m | y=%.3f m | yaw=%.1f deg (%.3f rad)",
        message.header.frame_id,
        message.pose.position.x,
        message.pose.position.y,
        math.degrees(yaw),
        yaw,
    )


def main():
    rospy.init_node("goal_coordinate_logger")
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback, queue_size=10)
    rospy.loginfo("Goal coordinate logger ready. Use RViz 2D Nav Goal.")
    rospy.spin()


if __name__ == "__main__":
    main()
