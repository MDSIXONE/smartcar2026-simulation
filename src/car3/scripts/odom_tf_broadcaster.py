#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class MapOdomBroadcaster:
    def __init__(self):
        rospy.init_node("odom_tf_broadcaster")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=20)

    def odom_callback(self, message):
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        if transform.header.stamp == rospy.Time():
            transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)


if __name__ == "__main__":
    try:
        MapOdomBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass