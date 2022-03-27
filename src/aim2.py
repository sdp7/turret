#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray 
from time import time, sleep 
 
# processes the data from the ROSTopic named "joint_states"
class aim:
    def __init__(self):
        rospy.init_node('move_arm',anonymous=True)
        self.rate = rospy.Rate(10)
        self.jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)    
        self.joint_pos = Float64MultiArray() 
        rospy.Subscriber("ball_position", Float64MultiArray, self.ball_callback)

    def joint_callback(self,data): 
        print("Msg: {}".format(data.header.seq)) 
        print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight: {0:.2f}rad\n\n".format(data.position[0],data.position[1])) 
        print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2: {0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist: {0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5])) 
        print("----------")

        #theta1 = data.position[3]
        #theta2 = data.position[6]
        #return (theta1, theta2)

    def ball_callback(self,data):

        camera_to_frame_x = 0.07
        camera_to_frame_z = -0.1

        x = data.data[0] + camera_to_frame_x
        y = -data.data[1]
        z = data.data[2] + camera_to_frame_x

        ball_position = (x,y,z)
        #print(ball_position)
        self.move_arm(ball_position)

    # if camera is mounted on turret
    def turret_to_wheelbase_frame(self,angles, ball_position):
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

    def calculate_angles(self,position):
        x = position[0]
        y = position[1]
        z = position[2]

        base_angle = math.atan2(y,x)

        # velocity of ejection
        v = 5
        # gravity constant
        g = 9.81
        # displacement horizontal
        d = math.sqrt(x*x + y*y)
        # height
        h = z - 0.08

        turret_angle = math.atan2(v**2 - math.sqrt(v**4 - g * (g * d**2 + 2 * v**2 * h)), g * d)

        return (base_angle, turret_angle)
    
    # listens to the "joint_states" topic and sends them to "joint_callback" for processing
    def read_joint_states(self): 
        rospy.Subscriber("joint_states",JointState, self.joint_callback)  
    
    # Makes sure the joints do not go outside the joint limits/break the servos
    def clean_joint_states(self,data): 
        lower_limits = [0, -3.14, -1.57, -1.57, -1, -1] 
        upper_limits = [0,  3.14,  1.57,  1.57,  1.57, 1] 
        clean_lower = np.maximum(lower_limits,data) 
        clean_upper = np.minimum(clean_lower,upper_limits) 
        return list(clean_upper) 
    
    # publishes a set of joint commands to the 'joint_trajectory_point' topic
    def move_arm(self,target_coordinate):
        #Joint Position vector should contain 6 elements:
        #[0, shoulder1, shoulder2, elbow, wrist, gripper]

        angles = self.calculate_angles(target_coordinate)
        base_angle = angles[0]
        turret_angle = angles[1]
        #print("angles",angles)
        #print(base_angle)
        #print(turret_angle)

        self.joint_pos.data = self.clean_joint_states([0, base_angle, 1.57, -1.47, turret_angle, 0])
        #print("joint pos data", self.joint_pos.data)
        self.jointpub.publish(self.joint_pos)
        self.read_joint_states()

 
#loops over the commands at 20Hz until shut down
if __name__ == '__main__': 
    aimer = aim()

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
