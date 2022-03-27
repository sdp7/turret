#!/usr/bin/env python
import rospy
import sys 
from numpy import maximum,minimum 
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray 
from time import time, sleep 
import parser

# processes the data from the ROSTopic named "joint_states"


def joint_callback(data): 
    print("Msg: {}".format(data.header.seq)) 
    print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight: {0:.2f}rad\n\n".format(data.position[0],data.position[1])) 
    print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2: {0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist: {0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5])) 
    print("Gripper Position:\n\tGripper: {0:.2f}rad\n".format(data.position[6])) 
    print("----------") 
 
# listens to the "joint_states" topic and sends them to "joint_callback" for processing
def read_joint_states(): 
    rospy.Subscriber("joint_states",JointState,joint_callback) 
 
# Makes sure the joints do not go outside the joint limits/break the servos
def clean_joint_states(data): 
    lower_limits = [0, -3.14, -1.57, -1.57, -1.57,   -1] 
    upper_limits = [0,  3.14,  1.57,  1.57,  1.57, 1.57] 
    clean_lower = maximum(lower_limits,data) 
    clean_upper = minimum(clean_lower,upper_limits) 
    return list(clean_upper) 
 # publishes a set of joint commands to the 'joint_trajectory_point' topic
def move_arm(baseAngle,turretAngle):
    #baseAngle = 45 #args.yaw
    #turretAngle = 45 #args.pitch
    jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)    
    joint_pos = Float64MultiArray() 
#   Joint Position vector should contain 6 elements:
#   [0, shoulder1, shoulder2, elbow, wrist, gripper]
    baseAngle = (baseAngle) * (1.57/90)
    turretAngle = turretAngle * (1.57/90) 
    joint_pos.data = clean_joint_states([0, baseAngle, 1.57, -1.47,turretAngle, 0])
   # joint_pos.data = clean_joint_states([0, baseAngle, 1.57, -1.5,turretAngle, -1.1]) 
    jointpub.publish(joint_pos) 
    read_joint_states() 
 
#loops over the commands at 20Hz until shut down
if __name__ == '__main__':
    #parser.add_argument("--base-angle", dest='yaw', type = Float64)
    #parser.add_argument('--turret-angle', dest='pitch', type = FLoat64)
    #args = parser.parse_args()
    rospy.init_node('move_arm',anonymous=True) 
    rate = rospy.Rate(10)
    i=-1
    while not rospy.is_shutdown(): 
        move_arm(180*i,20*i)
	i = -i
        sleep(2) 

