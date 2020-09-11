#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from rns_msgs.msg import JointCmd
import math


fromIncToRadFor10664 = 4096/(2*math.pi)
fromIncToRadFor12 = (1024*180)/(300*math.pi)
cmdPub = rospy.Publisher('cmd_joints',JointCmd,queue_size = 10)

def PublishCmdCallback(msgPos):
    global fromIncToRadFor10664, fromIncToRadFor12, cmdPub
    jointcmd = JointCmd()
    for i in range(len(msgPos.name)):
        jointcmd.channel = int(msgPos.name[i])
        if (int(msgPos.name[i]) == 0 or int(msgPos.name[i]) == 1 or int(msgPos.name[i]) == 2 or int(msgPos.name[i]) == 6 or int(msgPos.name[i]) == 7 or int(msgPos.name[i]) == 8):
            jointcmd.cmd = int(msgPos.position[i]*fromIncToRadFor10664)
        else:
            jointcmd.cmd = int(msgPos.position[i]*fromIncToRadFor12)
        cmdPub.publish(jointcmd)
        

if __name__ == '__main__':
    rospy.init_node('publish_cmd_in_rad_node',anonymous=True)
    rospy.Subscriber("/arm/cmd_arm_in_rad", JointState, PublishCmdCallback)
    rospy.spin()