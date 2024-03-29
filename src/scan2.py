#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from time import time, sleep

# CORRECT VERSION
 
# processes the data from the ROSTopic named "joint_states"
class scan:
    def __init__(self):
        rospy.init_node('scan_arm',anonymous=True)
        self.rate = rospy.Rate(2)
        rospy.init_node('scan2',anonymous=True)
        self.rate = rospy.Rate(10)
        self.intervals = 20
        self.counter = 0
        self.half_scan_done = False
        self.trajectory = self.generate_trajectory()
        self.jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)
        self.joint_pos = Float64MultiArray() 
        #rospy.Subscriber("fire_detection", Float64MultiArray, self.detect_callback)
        rospy.Subscriber("isFire", Bool, self.scan_callback)

    def joint_callback(self,data): 
        print("Msg: {}".format(data.header.seq)) 
        print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight: {0:.2f}rad\n\n".format(data.position[0],data.position[1])) 
        print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2: {0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist: {0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5])) 
        print("----------")

    def scan_callback(self,data):
        is_fire = data.data
        i = int(self.intervals/2)
        half_scan = [math.pi*factor for factor in np.linspace(0,-1,int(self.intervals/2))]
        half_scan.pop()
    
        # scan half pi
        if not self.half_scan_done:
            for i in range(len(half_scan)):
                if not is_fire:
                    self.move_arm(half_scan[i])
                if i == len(half_scan)-1:
                    self.half_scan_done = True
        # scan full circle
        elif not is_fire and self.half_scan_done:
            if self.counter == len(self.trajectory):
                self.counter = 0
            self.move_arm(self.trajectory[self.counter])
            self.counter += 1
        else:
            exit()
    
    # listens to the "joint_states" topic and sends them to "joint_callback" for processing
    def read_joint_states(self): 
        rospy.Subscriber("joint_states",JointState, self.joint_callback)  
    
    # makes sure the joints do not go outside the joint limits/break the servos
    def clean_joint_states(self,data): 
        lower_limits = [0, -3.5, -1.57, -1.57, -1, -1] 
        upper_limits = [0,  3.5,  1.57,  1.57,  1.57, 1] 
        clean_lower = np.maximum(lower_limits,data) 
        clean_upper = np.minimum(clean_lower,upper_limits) 
        return list(clean_upper)

    def generate_trajectory(self):
        clockwise_scan = [math.pi*factor for factor in np.linspace(-1,1,self.intervals)]
        anti_clockwise = clockwise_scan[::-1]
        clockwise_scan.pop()
        anti_clockwise.pop()
        trajectory = clockwise_scan + anti_clockwise
        return trajectory

    # publishes a set of joint commands to the 'joint_trajectory_point' topic
    def move_arm(self,angle):
        print(angle)
        #Joint Position vector should contain 6 elements:
        #[0, shoulder1, shoulder2, elbow, wrist, gripper]

        self.joint_pos.data = self.clean_joint_states([0, angle, 1.57, -1.47, 0, 0])
        self.jointpub.publish(self.joint_pos)
        self.read_joint_states()

 
#loops over the commands at 20Hz until shut down
if __name__ == '__main__': 
    scanner = scan()
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
