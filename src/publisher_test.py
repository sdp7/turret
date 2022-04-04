#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('fire_tester', Bool, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        fire = False
        rospy.loginfo(fire)
        pub.publish(fire)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
