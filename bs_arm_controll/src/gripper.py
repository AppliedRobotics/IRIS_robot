#!/usr/bin/env python
import roslib
import time
import rospy
from rns_msgs.msg import JointCmd
from std_msgs.msg import Bool

def grip_on_off(data):
    cmd_joint_pub = rospy.Publisher('cmd_joints',JointCmd,queue_size = 100)
    jointcmd = JointCmd()
    if(data.data):
        jointcmd.cmd = 660
        jointcmd.channel = 10
        time.sleep(0.01)
        cmd_joint_pub.publish(jointcmd)

        jointcmd.cmd = 380
        jointcmd.channel = 11
        time.sleep(0.01)
        cmd_joint_pub.publish(jointcmd)
    if(not data.data):
        jointcmd.cmd = 565
        jointcmd.channel = 10
        time.sleep(0.01)
        cmd_joint_pub.publish(jointcmd)

        jointcmd.cmd = 475
        jointcmd.channel = 11
        time.sleep(0.01)
        cmd_joint_pub.publish(jointcmd)


if __name__=='__main__':
    rospy.init_node('gipper',anonymous=True)
    print('Start gripper node')
    rospy.Subscriber("/arm/cmd_gripper", Bool, grip_on_off)
    rospy.spin()
