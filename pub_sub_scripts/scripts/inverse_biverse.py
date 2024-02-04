#!/usr/bin/env python3

import numpy as np
import math
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random as r
from sensor_msgs.msg import JointState
import rospy


def objective_function(joint_angles):
    d1 = 72
    d2 = 125
    d3 = 125
    d45 = 129.80

    angle1 = 0 # world to base is fixed
    angle2, angle3, angle4, angle5, angle6 = joint_angles

    #world_base

    A=([[ math.cos(angle1), -math.sin(angle1), 0, 0],
                [math.sin(angle1) , math.cos(angle1), 0, 0],
                [ 0, 0, 1, 0],
                [ 0, 0, 0, 1]])


    #base_shoulder

    B=([[ math.cos(angle2), 0, -math.sin(angle2), 0],
                    [ math.sin(angle2), 0, math.cos(angle2), 0],
                    [ 0, -1, 0, d1],
                    [ 0, 0, 0, 1]])


    #shoulder_elbow

    C=([[ math.cos(angle3), -math.sin(angle3), 0, (d2*math.cos(angle3))],
                    [ math.sin(angle3), math.cos(angle3), 0, (d2*math.sin(angle3))],
                    [ 0, 0, 1, 0],
                    [ 0, 0, 0, 1]])

    #elbow_wrist

    D=([[ math.cos(angle4), -math.sin(angle4), 0, (d3*math.cos(angle4))],
                [ math.sin(angle4), math.cos(angle4), 0, (d3*math.sin(angle4))],
                [ 0, 0, 1, 0],
                [ 0, 0, 0, 1]])


    #wrist_twist

    E=([[ math.cos(angle5), 0, -math.sin(angle5), 0],
        [ math.sin(angle5), 0, math.cos(angle5), 0],
        [ 0, -1, 0, 0],
        [ 0, 0, 0, 1]])


    #twist_ee

    F=([[ math.cos(angle6), -math.sin(angle6), 0, 0],
            [ math.sin(angle6), math.cos(angle6), 0, 0],
            [ 0, 0, 1, d45],
            [ 0, 0, 0, 1]])

    H=([0],[0],[0],[1])

    result_matrix = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(A, B), C), D), E), F),H)

    difference = goal_pos-result_matrix
    error = abs(difference[0] + difference[1] + difference[2])

    print("Error is: ",error)
    print("result is: ",result_matrix)
    #return(error)
    return(np.linalg.norm(difference))

 

     

def plot(joint_angles):
    d1 = 72
    d2 = 125
    d3 = 125
    d45 = 129.80

    angle1 = 0 # world to base is fixed
    angle2, angle3, angle4, angle5, angle6 = joint_angles

 
    #world_base

    A=([[ math.cos(angle1), -math.sin(angle1), 0, 0],
                [math.sin(angle1) , math.cos(angle1), 0, 0],
                [ 0, 0, 1, 0],
                [ 0, 0, 0, 1]])
 

    #base_shoulder

    B=([[ math.cos(angle2), 0, -math.sin(angle2), 0],
                    [ math.sin(angle2), 0, math.cos(angle2), 0],
                    [ 0, -1, 0, d1],
                    [ 0, 0, 0, 1]])


    #shoulder_elbow

    C=([[ math.cos(angle3), -math.sin(angle3), 0, (d2*math.cos(angle3))],
                    [ math.sin(angle3), math.cos(angle3), 0, (d2*math.sin(angle3))],
                    [ 0, 0, 1, 0],
                    [ 0, 0, 0, 1]])

    #elbow_wrist
    D=([[ math.cos(angle4), -math.sin(angle4), 0, (d3*math.cos(angle4))],
                [ math.sin(angle4), math.cos(angle4), 0, (d3*math.sin(angle4))],
                [ 0, 0, 1, 0],
                [ 0, 0, 0, 1]])


    #wrist_twist

    E=([[ math.cos(angle5), 0, -math.sin(angle5), 0],
        [ math.sin(angle5), 0, math.cos(angle5), 0],
        [ 0, -1, 0, 0],
        [ 0, 0, 0, 1]])
 
    #twist_ee

    F=([[ math.cos(angle6), -math.sin(angle6), 0, 0],
            [ math.sin(angle6), math.cos(angle6), 0, 0],
            [ 0, 0, 1, d45],
            [ 0, 0, 0, 1]])

   
    H=([0],[0],[0],[1])

 
    # Plot robot arm
    fig = plt.figure()
    ax = plt.axes( projection='3d')

    zeros= [0,0,0]
    ax.plot(zeros,zeros,zeros,marker="o")
    first = np.dot(np.dot(A, B),H)
    ax.plot(first[0],first[1],first[2], marker="o",)
    second = np.dot(np.dot(np.dot(A, B),C),H)
    ax.plot(second[0],second[1],second[2], marker="o",)
    third = np.dot(np.dot(np.dot(np.dot(A, B),C),D),H)
    ax.plot(third[0],third[1],third[2], marker="o",)
    four = np.dot(np.dot(np.dot(np.dot(np.dot(A, B),C),D),E),H)
    ax.plot(four[0],four[1],four[2], marker="o",)
    five = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(A, B),C),D),E),F),H)
    ax.plot(five[0],five[1],five[2], marker="x",)
    ax.set_xlim([-400, 400])
    ax.set_ylim([-400, 400])
    ax.set_zlim([0,400])

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('Robot Arm Visualization')
    ax.legend()
    plt.show()

 
def joint_converstion(joint):

    angle2,angle3,angle4,angle5,angle6 = joint
    nangle2 =   angle2 - 1.5707
    nangle3 =  -angle3
    nangle4 =  -angle4 + 1.5707
    nangle5 =  -angle5
    nangle6 =   angle6

    result = [nangle2 ,nangle3 ,nangle4 ,nangle5, angle6, 0,0]
    print(result)
    return(result)
    


rospy.init_node("IK_JointStates")




joint_limits = [
    (0,3.1416),
    (-3.1416,0),
    (-1.5708,1.5708),
    (-3.1416,0),
    (-10000000,150000708), #(-1.5708,1.5708),
] 

#initial_guess = [1.5, -1.5, 0, -1.5, 0]
initial_guess = [1.5707,-0.707,0.7,0,1]
#goal_pos= ([376],[0.1],[71.90],[1])
goal_pos= ([0],[250],[0],[1])
count = 0
# Use the solver to find joint angles (uncommnet below for solver)
result = minimize(objective_function, initial_guess, method='Nelder-Mead', tol=6e-8, bounds=joint_limits,)
# Extract the solution (uncommnet below for solver)
final_joint_angles = result.x
print("joint angles are: ",result.x)
zeros = [1.5707,-0.707,0.7,0,1]
#plot(result.x)
print(result.fun)

 
#USE TO RANDOMISE STARTING POSITIONS

# while result.fun > 1.0 :

#     initial_guess = [r.uniform(0,3.1416),r.uniform(-3.1416,0),r.uniform(-1.5708,1.5708),r.uniform(-3.1416,0),r.uniform(-10000000,150000708)]
#     result = minimize(objective_function, initial_guess, method='Nelder-Mead', tol=6e-8, bounds=joint_limits,)
#     # Extract the solution (uncommnet below for solver)
#     final_joint_angles = result.x
#     print("joint angles are: ",result.x)
#     zeros = [1.5707,-0.707,0.7,0,1]
#     #plot(result.x)
#     print(result.fun)
#     count += 1

#     if count == 10:
#         break



#plot(result.x)
#joint_converstion(result.x)
#plot(result.x)

#making joint sate message
while not rospy.is_shutdown():
    msg = JointState()

    msg.name = ["base_joint","shoulder_joint"
   ,"elbow_joint"
   ,"wrist_pitch_joint"
   ,"wrist_roll_joint"
   ,"gripper_joint"
   ,"sub_gripper_joint]"]
    
    msg.position = joint_converstion(result.x)
    msg.velocity = []
    msg.effort = []
    
            
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ''

    pub = rospy.Publisher("joint_states",JointState,queue_size=10)
    pub.publish(msg)

    rospy.sleep(1.0)

   