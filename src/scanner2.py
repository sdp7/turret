#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray, Bool
from std_msgs.msg import Bool
from time import time, sleep
import sys

# CORRECT VERSION

# processes the data from the ROSTopic named "joint_states"
class scan:
    def __init__(self):
        rospy.init_node('scan_arm_real',anonymous=True)
        
        self.rate = rospy.Rate(20)
        self.stop = rospy.Rate(1)
        self.intervals = 20
        
        self.counter = 0
        self.half_scan_done = False
        self.trajectory = self.generate_trajectory()
        self.half_trajectory = self.generate_half_trajectory()
        
        self.jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)
        self.statepub = rospy.Publisher('scan_state', Bool, queue_size =10)
        
        self.joint_pos = Float64MultiArray() 
        self.scan = True
        
        rospy.Subscriber("isFire", Bool, self.read_fire_callback)
        
        #rospy.Subscriber("fire_tester", Bool, self.scan_callback)

    def joint_callback(self,data): 
        print("Msg: {}".format(data.header.seq)) 
        print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight: {0:.2f}rad\n\n".format(data.position[0],data.position[1])) 
        print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2: {0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist: {0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5])) 
        print("----------")
    
    # listens to the "joint_states" topic and sends them to "joint_callback" for processing
    def read_fire_callback(self, fire_data): 
        # rospy.Subscriber("isFire", Bool)  
        self.stop.sleep()
        data = fire_data.data
        self.scan_callback(data)

    def scan_callback(self, fire_data):
        is_fire = fire_data
        
        # self.scan = not is_fire
        # self.statepub.publish(self.scan)
        
        if is_fire == True:
            print("Found fire")
            self.scan = False
            rospy.signal_shutdown()
            self.statepub.publish(self.scan)

        # scan half pi
        elif not(self.half_scan_done or is_fire):
            self.move_arm(self.half_trajectory[self.counter])
            self.counter += 1
            if self.counter == len(self.half_trajectory):
                self.counter = 0
                self.half_scan_done = True
            
        # scan full circle
        if (not is_fire) and self.half_scan_done:
            if self.counter == len(self.trajectory):
                self.counter = 0
            self.move_arm(self.trajectory[self.counter])
            self.counter += 1
        self.statepub.publish(self.scan)

    # listens to the "joint_states" topic and sends them to "joint_callback" for processing
    def read_joint_states(self): 
        rospy.Subscriber("joint_states",JointState, self.joint_callback)  

    # makes sure the joints do not go outside the joint limits/break the servos
    def clean_joint_states(self,data): 
        lower_limits = [0, -3.141, -1.57, -1.57, -1, -1] 
        upper_limits = [0,  3.141,  1.57,  1.57,  1.57, 1] 
        clean_lower = np.maximum(lower_limits,data) 
        clean_upper = np.minimum(clean_lower,upper_limits) 
        return list(clean_upper)

    def generate_trajectory(self):
        clockwise_scan = [3.14*factor for factor in np.linspace(-1, 1, self.intervals)]
        anti_clockwise = clockwise_scan[::-1]
        clockwise_scan.pop()
        anti_clockwise.pop()
        trajectory = clockwise_scan + anti_clockwise
        print(trajectory)
        return trajectory

    def generate_half_trajectory(self):
        stops = int(self.intervals/2)
        trajectory = [3.14*factor for factor in np.linspace(0, -1, stops, endpoint = False)]
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
            # if scanner.scan == False: break
        except KeyboardInterrupt:
            print("Shutting down")
