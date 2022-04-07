#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float64MultiArray


# def talker():
#     pub = rospy.Publisher('isFire', Bool, queue_size=10)
#     rospy.init_node('publisher', anonymous=True)
#     rate = rospy.Rate(2)

class talker():
    def __init__(self):
        self.pub = rospy.Publisher('fire_tester', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(50)

    def pub(self):
        fire = Float64MultiArray
        rospy.loginfo(fire)
        self.pub.publish(fire)
        self.rate.sleep()

def main():
    rospy.init_node('publisher', anonymous=True)
    t = talker()
    while not rospy.is_shutdown():
        try:
            t.pub()
        except rospy.ROSInterruptException:
            break
    print("Shutting down fake fire")


if __name__ == '__main__':
    main()