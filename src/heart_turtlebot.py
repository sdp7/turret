#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray 
from time import time, sleep 
 
# processes the data from the ROSTopic named "joint_states"
def joint_callback(data): 
    print("Msg: {}".format(data.header.seq)) 
    print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight: {0:.2f}rad\n\n".format(data.position[0],data.position[1])) 
    print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2: {0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist: {0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5])) 
    print("----------")

    #theta1 = data.position[3]
    #theta2 = data.position[6]

    #return (theta1, theta2)

def ball_callback(data):
    x = data[0]
    y = data[1]
    z = data[2]

    ball_position = (x,y,z)

    return ball_position

def turret_to_wheelbase_frame(angles, ball_position):
    theta1 = angles[0]
    theta2 = angles[1]

    s1 = np.sin(theta1)
    s2 = np.sin(theta2)

    c1 = np.sin(theta1)
    c2 = np.sin(theta2)

    a11 = c1 * s2
    a12 = c1 * c2
    a13 = -s1
    a14 = 0.03 * c1 * s2 + 0.06 * c1 * c2

    a21 = s1 * s2
    a22 = s1 * c2
    a23 = c1
    a24 = 0.03 * s1 * s2 + 0.06 * s1 * c2

    a31 = c2
    a32 = -s2
    a33 = 0
    a34 = 0.03 * c2 + 0.08 - 0.06 * s2

    a_0_3 = np.array([[a11, a12, a13, a14], [a21, a22, a23, a24], [a31, a32, a33, a34], [0, 0, 0, 1]])
    a_3_0 = np.linalg.pinv(a_0_3)

    ball_homogenous_coord = np.array([[ball_position[0], ball_position[1], ball_position[2]]]).T

    base_position = a_3_0 * ball_homogenous_coord

    return base_position

def calculate_angles(position):
    x = position[0]
    y = position[1]
    z = position[2]

    base_angle = math.atan2(y,x)

    # velocity of ejection
    v = 100
    # gravity constant
    g = 9.81
    # displacement horizontal
    d = math.sqrt(x*x + y*y)
    # height
    h = z - 0.08

    turret_angle = math.atan2(v**2 - math.sqrt(v**4 - g * (g * d**2 + 2 * v**2 * h)), g * d)

    return (base_angle, turret_angle)
 
# listens to the "joint_states" topic and sends them to "joint_callback" for processing
def read_joint_states(): 
    rospy.Subscriber("joint_states",JointState,joint_callback) 
    #rospy.Subscriber("ball_pos",Float64MultiArray,ball_callback) 
 
# Makes sure the joints do not go outside the joint limits/break the servos
def clean_joint_states(data): 
    lower_limits = [0, -3.14, -1.57, -1.57, -1, -1] 
    upper_limits = [0,  3.14,  1.57,  1.57,  1.57, 1] 
    clean_lower = np.maximum(lower_limits,data) 
    clean_upper = np.minimum(clean_lower,upper_limits) 
    return list(clean_upper) 
 
# publishes a set of joint commands to the 'joint_trajectory_point' topic
def move_arm(target_coordinate):
    jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)    
    joint_pos = Float64MultiArray() 
#   Joint Position vector should contain 6 elements:
#   [0, shoulder1, shoulder2, elbow, wrist, gripper]

    angles = calculate_angles(target_coordinate)

    base_angle = angles[0]
    turret_angle = angles[1]

    joint_pos.data = clean_joint_states([0, base_angle, 1.57, -1.47,turret_angle, 0])
    jointpub.publish(joint_pos) 
    read_joint_states()

def heart():
    curve1 = [(x,math.sqrt(1-(abs(x)-1)**2)) for x in np.linspace(0,2,10)]
    curve2 = [(x,math.acos(1-abs(x))-3.14) for x in np.linspace(2,-2, 20)]
    curve3 = [(x,math.sqrt(1-(abs(x)-1)**2)) for x in np.linspace(-2,0,10)]
    heart_curve = curve1 + curve2 + curve3

    return heart_curve
 
#loops over the commands at 20Hz until shut down
if __name__ == '__main__': 
    rospy.init_node('move_arm',anonymous=True) 
    rate = rospy.Rate(30)
    i=0 
    heart_curve = heart()
    while not rospy.is_shutdown():

        if i == len(heart_curve):
            i = 0 
        point = (0.6, heart_curve[i][0]/5, heart_curve[i][1]/5)
 
        move_arm(point)
        i+=1
        sleep(0.1)
