#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32MultiArray
from math import sin
def talker():
    pub = rospy.Publisher('cmd_joints', Int32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(40) # 10hz
    i = 0
    while not rospy.is_shutdown():
        positions = Int32MultiArray()
        positions.data = [abs(int(1023*sin(i))),160,800,abs(int(800*sin(i))),500]
        i = i + 0.01
        rospy.loginfo(positions)
        pub.publish(positions)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass