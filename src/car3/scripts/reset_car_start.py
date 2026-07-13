#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState


def main():
    rospy.init_node("reset_car_start")
    model_name = rospy.get_param("~model_name", "car3")
    start_x = rospy.get_param("~start_x", 0.0)
    start_y = rospy.get_param("~start_y", 0.0)
    start_z = rospy.get_param("~start_z", 0.01)
    start_yaw = rospy.get_param("~start_yaw", 0.0)

    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/set_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    rate = rospy.Rate(10)
    for _ in range(100):
        if rospy.is_shutdown():
            return
        if get_state(model_name, "world").success:
            break
        rate.sleep()
    else:
        rospy.logerr("reset_car_start: model %s not found", model_name)
        return

    state = ModelState()
    state.model_name = model_name
    state.reference_frame = "world"
    state.pose.position.x = start_x
    state.pose.position.y = start_y
    state.pose.position.z = start_z
    state.pose.orientation.z = __import__("math").sin(start_yaw / 2.0)
    state.pose.orientation.w = __import__("math").cos(start_yaw / 2.0)
    response = set_state(state)
    if response.success:
        rospy.loginfo("reset_car_start: %s reset to (%.3f, %.3f, %.3f)", model_name, start_x, start_y, start_z)
    else:
        rospy.logerr("reset_car_start: %s", response.status_message)


if __name__ == "__main__":
    main()
