#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from LeftRoboticArmClass import LeftRoboticArm
from RightRoboticArmClass import RightRoboticArm

def ParseCmd(msgText):
    try:
        roll = 0
        msgList = msgText.split()
        arm = msgList[0]
        x = float(msgList[1])
        y = float(msgList[2])
        z = float(msgList[3])
        roll = float(msgList[4])
    except ValueError:
        rospy.logerr("Value Input Error")
    except IndexError:
        rospy.logerr("Index Input Error")
    return arm,x,y,z,roll

def ChangeOffset(solve,roll,roboticArm):
    position = []
    for i in range(3):
        position.append(solve[i+3]*roboticArm.motorSign[i]+ roboticArm.motorOffset[i])
    position.append(roll*roboticArm.motorSign[3]+ roboticArm.motorOffset[3])
    return position

def MainToPointCallback(msg):
    arm, x, y, z, roll = ParseCmd(msg.data)

    jointPub = rospy.Publisher("/arm/cmd_arm_in_rad", JointState, queue_size= 10)
    jointMsg = JointState()

    if(arm == 'l'):
        roboticArm = LeftRoboticArm()
    elif(arm == 'r'):
        roboticArm = RightRoboticArm()
    else:
        rospy.logerr('Unknown Device')
        return
    solve, err = roboticArm.SolveIP(x,y,z)
    if (type(solve) is int):
        rospy.loginfo('Point can\'t be reached' )
    else:
        rospy.loginfo('Point can be reached')
        jointMsg.name = [str(roboticArm.motorId[0]),str(roboticArm.motorId[1]),str(roboticArm.motorId[2]),str(roboticArm.motorId[3])]
        jointMsg.position = ChangeOffset(solve,roll,roboticArm)
        jointMsg.header.stamp = rospy.Time.now()
        jointPub.publish(jointMsg)

if __name__ == '__main__':
    rospy.init_node('main_to_point_node',anonymous=True)
    rospy.Subscriber("/arm/cmd_arm_bs", String, MainToPointCallback)
    rospy.spin()