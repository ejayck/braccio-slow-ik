#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from inverse_biverse import IK_solver


def callback(data):
    ik = IK_solver()
    position = data.pose.position
    x = position.x * 1000
    y = position.y * 1000
    z = position.z * 1000
    print([x,y,z,1])
    #ik.solve_IK([x,y,z,1])
    ik.solve_IK(([x],[y],[z],[1]))


rospy.init_node("goal_listener")
while not rospy.is_shutdown():

    rospy.Subscriber("move_base_simple/goal",PoseStamped,callback=callback)
    rospy.spin()
