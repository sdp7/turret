#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from time import time, sleep
import message_filters

# CORRECT VERSION
 
# processes the data from the ROSTopic named "joint_states"
class scan:
    def __init__(self):
        global r
        r = 50
        self.rate = rospy.Rate(r)
        global sl
        sl = 1/r
        
        self.intervals = 40
        self.counter = 0
        self.half_scan_done = False
        
        self.trajectory = self.generate_trajectory()
        self.half_trajectory = self.generate_half_trajectory()
        
        self.jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size=10)
        self.joint_pos = Float64MultiArray() 
        
        # self.fire_sub = message_filters.Subscriber("isFire", Float64MultiArray, queue_size=10)
        self.joint_sub = message_filters.Subscriber("joint_states", JointState) 
        
        # fake fire publisher
        self.fire_sub = rospy.Subscriber("fire_tester", Float64MultiArray, self.detect_callback)

        self.sub_list = [self.joint_sub, self.fire_sub]

        # synchronise topics for isFire and jointStates so we have the right joint angles when finding isFire, 
        # slop = delay in sec in messages is syncesd
        self.joint_sync = message_filters.ApproximateTimeSynchronizer(self.sub_list, queue_size=10, slop = sl, allow_headerless=True)
        self.joint_sync.registerCallback(self.detect_callback)
<<<<<<< HEAD
        
        # rospy.Subscriber("isFire", Bool, self.scan_callback)
=======
>>>>>>> demo3

    def joint_callback(self,data): 
        print("Msg: {}".format(data.header)) 
        b, s, e, w, g, end = round(data.position[2],3), round(data.position[3],3), round(data.position[4],3), round(data.position[5],3), round(data.position[6],3), round(data.position[7],3)
        print(f"Joint Positions: \n [Base]: {b} \n [Shoulder]: {s} ~ [Elbow]: {e}\n [Wrist]: {w} | [Gripper]: {g}\n ")
        # print(f"complete joints data{[data.position]}")
        # print("----------")


    def detect_callback(self, joint_data, fire_data):
        print('in detect_callback')
        
        print("Msg: {}".format(joint_data.header)) 
        b, s, e, w, g, end = round(joint_data.position[2],3), round(joint_data.position[3],3), round(joint_data.position[4],3), round(joint_data.position[5],3), round(joint_data.position[6],3), round(joint_data.position[7],3)
        print(f"Joint Positions: \n [Base]: {b} \n [Shoulder]: {s} ~ [Elbow]: {e}\n [Wrist]: {w} | [Gripper]: {g}\n ")

        is_fire = len(fire_data.data)

        if is_fire > 0:
            fire_x = fire_data.data[0]
            fire_y = fire_data.data[0]
            self.joint_pos = joint_data.data
            print(f'detect callback, found fire at {fire_x,fire_y}, exiting scanning')
            exit()

        else:
            print(f"detect callback, didn't find fire at")
            self.scan_callback(joint_data)
        # self.rate.sleep()

    def scan_callback(self,joint_data):
        print('in scan_callback')
        # scan half pi
        if not(self.half_scan_done):
            print(f'scan 1/2 pi')
            self.move_arm(self.half_trajectory[self.counter])
            self.counter += 1
            if self.counter == len(self.half_trajectory):
                self.counter = 0
                self.half_scan_done = True
        
        # scan full circle
        if self.half_scan_done:
            print(f'scan pi')
            if self.counter == len(self.trajectory):
                self.counter = 0
            self.move_arm(self.trajectory[self.counter])
            self.counter += 1
        # rospy.sleep(0.05)

    
    # listens to the "joint_states" topic and sends them to "joint_callback" for processing
    def read_joint_states(self): 
        rospy.Subscriber("joint_states",JointState, self.joint_callback)  
    
    # makes sure the joints do not go outside the joint limits/break the servos
    def clean_joint_states(self,data): 
        print('cleaning joint state')
        lower_limits = [0, -3.141, -1.57, -1.57, -1, -1] 
        upper_limits = [0,  3.141,  1.57,  1.57,  1.57, 1] 
        clean_lower = np.maximum(lower_limits,data) 
        clean_upper = np.minimum(clean_lower,upper_limits) 
        return list(clean_upper)

    def generate_trajectory(self):
        print('generate trajectory')
        clockwise_scan = [3.14*factor for factor in np.linspace(-1, 1, self.intervals)]
        anti_clockwise = clockwise_scan[::-1]
        clockwise_scan.pop()
        anti_clockwise.pop()
        trajectory = clockwise_scan + anti_clockwise
        print(f"trajector is: {trajectory}")
        return trajectory

    def generate_half_trajectory(self):
        print('generate half trajectory')
        stops = int(self.intervals/2)
        trajectory = [3.14*factor for factor in np.linspace(0, -1, stops, endpoint = False)]
        print(f"trajector is: {trajectory}")
        return trajectory

    # publishes a set of joint commands to the 'joint_trajectory_point' topic
    def move_arm(self,angle):
        print(f'asked to move arm by {angle}')
        
        #Joint Position vector should contain 6 elements:
        #[0, shoulder1, shoulder2, elbow, wrist, gripper]
        self.joint_pos.data = self.clean_joint_states([0, angle, 1.57, -1.47, 0, 0])
        self.jointpub.publish(self.joint_pos)
        # self.read_joint_states()

def main():
    rospy.init_node('scanner')
    scanner = scan()
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down scanner")
            break
 
#loops over the commands at 20Hz until shut down
if __name__ == '__main__': 
    main()
