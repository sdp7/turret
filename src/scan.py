#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray 
from time import time, sleep 
 
# processes the data from the ROSTopic named "joint_states"
class scan:
    def __init__(self):
        rospy.init_node('scan_arm',anonymous=True)
        global r
        r = 1
        self.rate = rospy.Rate(r)
        self.joint_sub = rospy.Subscriber("joint_states", JointState, self.scan_callback)
        self.joint_pub = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size = 10)    
        self.joint_pos = Float64MultiArray() 
        global pi 
        pi = 3.14
        global n_pi
        n_pi = -3.14
        self.right = True

    def scan_callback(self,data):
        print("Msg: {}".format(data.header.seq)) 
        b, s, e, w, g, end = data.position[2], data.position[3], data.position[4], data.position[5], data.position[6], data.position[7]
        print(f"Joint Positions: \n [Base]: {b} rads | [Shouler]: {s} rads [Elbow]: {e}\n  rad | [Wrist]: {w} rad | [Gripper]: {g}rad\n")
        #print(f"complete joints data{[data.position]}")
        print("----------")

        #base_angle = 1.0
        base_angle = self.calculate_base(b)
        
        #wrist
        turret_angle = 0.0
        
        self.joint_pos.data = self.clean_joint_states([base_angle, s, e, w, g, end]) 
        self.joint_pub.publish(self.joint_pos)
        self.rate.sleep()

    def calculate_base(self,x):
        vel = pi/(10)
        print(f"calc_base got {x} and vel is {vel}\n")
        
        if (n_pi<x<pi) and (self.right): #its in bewtween -pi and pi and turning right
            print("its in bewtween -pi and pi and turning right")
            x =+ vel 

        elif (n_pi<x<pi) and (not self.right): #its in bewtween -pi and pi and turning left
            print("its in bewtween -pi and pi and turning left")
            x =- vel  

        elif (x>=pi) and (self.right): #need to switch dir to left
            print("need to switch dir to right")
            self.right = False
            x =- vel
        
        elif (n_pi<=x) and (self.right): #need to switch dir to right
            print("need to switch dir to right")
            self.right = False
            x =+ vel
        else:
            x = 0

        print(f"new base angle should be: {x}\n")
        return x
    
    # Makes sure the joints do not go outside the joint limits/break the servos
    def clean_joint_states(self,data): 
        lower_limits = [-3.14, -3.14, -1.57, -1.57, -1, -1] 
        upper_limits = [3.14,  3.14,  1.57,  1.57,  1.57, 1] 
        clean_lower = np.maximum(lower_limits,data) 
        clean_upper = np.minimum(clean_lower,upper_limits) 
        return list(clean_upper) 
 
#loops over the commands at 20Hz until shut down
if __name__ == '__main__': 
    sc = scan()

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
