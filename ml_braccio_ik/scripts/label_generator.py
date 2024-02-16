#!/usr/bin/env python3

import math
import rospy
import numpy as np
import random as r
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

d1 = 72
d2 = 125
d3 = 125
d45 = 129.80

angle1 = 0 # world to base is fixed
angle2, angle3, angle4, angle5, angle6 = joint_angles

#world_base

self.A=([[ math.cos(angle1), -math.sin(angle1), 0, 0],
            [math.sin(angle1) , math.cos(angle1), 0, 0],
            [ 0, 0, 1, 0],
            [ 0, 0, 0, 1]])
#base_shoulder
self.B=([[ math.cos(angle2), 0, -math.sin(angle2), 0],
                [ math.sin(angle2), 0, math.cos(angle2), 0],
                [ 0, -1, 0, d1],
                [ 0, 0, 0, 1]])
#shoulder_elbow
self.C=([[ math.cos(angle3), -math.sin(angle3), 0, (d2*math.cos(angle3))],
                [ math.sin(angle3), math.cos(angle3), 0, (d2*math.sin(angle3))],
                [ 0, 0, 1, 0],
                [ 0, 0, 0, 1]])
#elbow_wrist
self.D=([[ math.cos(angle4), -math.sin(angle4), 0, (d3*math.cos(angle4))],
            [ math.sin(angle4), math.cos(angle4), 0, (d3*math.sin(angle4))],
            [ 0, 0, 1, 0],
            [ 0, 0, 0, 1]])
#wrist_twist
self.E=([[ math.cos(angle5), 0, -math.sin(angle5), 0],
    [ math.sin(angle5), 0, math.cos(angle5), 0],
    [ 0, -1, 0, 0],
    [ 0, 0, 0, 1]])
#twist_ee
self.F=([[ math.cos(angle6), -math.sin(angle6), 0, 0],
        [ math.sin(angle6), math.cos(angle6), 0, 0],
        [ 0, 0, 1, d45],
        [ 0, 0, 0, 1]])
self.H=([0],[0],[0],[1])

result_matrix = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(self.A, self.B), self.C), self.D), self.E), self.F),self.H)

difference = self.goal_pos-result_matrix
error = abs(difference[0] + difference[1] + difference[2])

print("Error is: ",error)
print("result is: ",result_matrix)
#return(error)
return(np.linalg.norm(difference))