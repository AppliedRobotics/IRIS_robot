#!/usr/bin/env python
import rospy  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64 
import math
from LeftRoboticArmClass import LeftRoboticArm
from RightRoboticArmClass import RightRoboticArm

fromEncToRadFor1066428 = 4096/(2*math.pi)
fromEncToRadFor12 = (1024*180)/(300*math.pi)
rRA = RightRoboticArm()
lRA = LeftRoboticArm()


def read_pose_callback(data_list):
    global fromEncToRadFor12, fromEncToRadFor1066428, rRA, lRA

    joint_state = JointState()

    joint_state.name.append('0')
    joint_state.position.append(round(-data_list.position[0]/fromEncToRadFor1066428 + lRA.motorOffset[0],2))

    joint_state.name.append('1')
    joint_state.position.append(round(data_list.position[1]/fromEncToRadFor1066428 - lRA.motorOffset[1],2))

    joint_state.name.append('2')
    joint_state.position.append(round(data_list.position[2]/fromEncToRadFor1066428 - lRA.motorOffset[2],2))

    joint_state.name.append('3')
    joint_state.position.append(round(data_list.position[3]/fromEncToRadFor12 - lRA.motorOffset[3],2))

    joint_state.name.append('4')
    joint_state.position.append(round(data_list.position[4]/fromEncToRadFor12,2))

    joint_state.name.append('5')
    joint_state.position.append(round(data_list.position[5]/fromEncToRadFor12,2))


    joint_state.name.append('6')
    joint_state.position.append(round(-data_list.position[6]/fromEncToRadFor1066428 + rRA.motorOffset[0],2))

    joint_state.name.append('7')
    joint_state.position.append(round(-data_list.position[7]/fromEncToRadFor1066428 + rRA.motorOffset[1],2))

    joint_state.name.append('8')
    joint_state.position.append(round(-data_list.position[8]/fromEncToRadFor1066428 + rRA.motorOffset[2],2))

    joint_state.name.append('9')
    joint_state.position.append(round(-data_list.position[9]/fromEncToRadFor12 + rRA.motorOffset[3],2))

    joint_state.name.append('10')
    joint_state.position.append(round(data_list.position[10]/fromEncToRadFor12,2))

    joint_state.name.append('11')
    joint_state.position.append(round(data_list.position[11]/fromEncToRadFor12,2))


    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    joint_state.header.stamp = rospy.Time.now()
    joint_pub.publish(joint_state)

if __name__ == '__main__':
    rospy.init_node('Read_joint_state_node')
    rospy.loginfo('Read_joint_state node was started')
    rospy.Subscriber('arm_joint_states', JointState, read_pose_callback)
    rospy.spin()