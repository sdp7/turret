#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float64MultiArray, Float64
from time import time, sleep 


base_to_turret = 0.13 # 130.0 mm
turret_to_camera = 0.05 # 50 mm

camera_to_base_height = 0.18  # 180 mm
camera_to_base_forward = 0.055 # 55 mm

camera_to_frame_x = 0.07
camera_to_frame_z = -0.1
 
# processes the data from the ROSTopic named "joint_states"
class aim:
    def __init__(self):
        rospy.init_node('move_arm',anonymous=True)
        self.rate = rospy.Rate(10)
        self.jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)
        self.current_pos = [0, 0]
        self.joint_pos = Float64MultiArray()
        self.depth = 0
        rospy.Subscriber("fire_position", Float64MultiArray, self.fire_callback)
        rospy.Subscriber("joint_states",JointState, self.joint_callback)
        rospy.Subscriber("Sonar", Float64, self.depth_callback)

    def depth_callback(self, data):
        self.depth = data.data

    def joint_callback(self,data):
        base_angle = data.position[2]
        turret_angle = data.position[5]
        self.current_pos = [base_angle, turret_angle]

    def to3D(self, x, y, distance, rectangle_width):
        focal_length = 0.0031
        sensor_size_horizontal = 0.00358   # 3.58 x 2.02 mm

        image_size = (sensor_size_horizontal * rectangle_width) / 1280
        object_size = (distance * image_size) / focal_length

        pixel_to_meter = object_size/rectangle_width

        z_meter = y * pixel_to_meter
        y_meter = x * pixel_to_meter
        x_meter = distance
        return (x_meter, y_meter, z_meter)

    def ball_callback(self,data):


        x_ = 1
        y_ = -data.data[1]
        z_ = data.data[2]

        coordinates = to3D(self, x_, y_, z_, 1)

        fire_position = (x,y,z)
        self.move_arm(fire_position)

    def rotate_around_z(x, y, z, theta):
        sin = np.sin(theta)
        cos = np.cos(theta)

        rotation_matrix = np.array[[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]]
        coordinates_from_camera = np.array[[x, y, z]]

        coordinates = np.matmul(rotation_matrix, coordinates_from_camera)
        return coordinates


    def calculate_angles(self,position):
        x = position[0] - 0.035 # turret to camera has a forward displacement of 3.5 cm
        y = position[1]
        z = position[2] + 0.05 # turret to camera has a vertical displacement of 5 cm

        base_angle = math.atan2(y,x) + self.current_pos[0]

        # velocity of ejection
        v = 5
        # gravity constant
        g = 9.81
        # displacement horizontal
        d = math.sqrt(x*x + y*y)
        # height
        h = z

        turret_angle = math.atan2(v**2 - math.sqrt(v**4 - g * (g * d**2 + 2 * v**2 * h)), g * d) + self.current_pos[1]

        return (base_angle, turret_angle)
    
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

        self.joint_pos.data = self.clean_joint_states([0, base_angle, 1.57, -1.47, turret_angle, 0])
        #print("joint pos data", self.joint_pos.data)
        self.jointpub.publish(self.joint_pos)

 
#loops over the commands at 20Hz until shut down
if __name__ == '__main__': 
    aimer = aim()

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")