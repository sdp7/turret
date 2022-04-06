#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('isFire', Bool, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(2)
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
