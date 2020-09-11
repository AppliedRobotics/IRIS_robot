#!/usr/bin/env python



import rospy
from sensor_msgs.msg import JointState
from time import sleep

def clean_poses(poses):
    new_poses = []
    for pose in poses:
        if(pose != '' and pose != '\n'):
            new_poses.append(float(pose))
    return new_poses

if __name__=="__main__":
    rospy.init_node('do_dance')
    pubposes = rospy.Publisher('/arm/cmd_arm_in_pos',JointState,queue_size=10)
    msg = JointState()
    msg.name = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11']
    file = open('/home/iris/iris_ws/src/dance/src/movements.txt', 'r')
    while not rospy.is_shutdown():
        s = file.readline()
        while(s != ''):
            s = file.readline()
            poses = s.split(',')
            poses = clean_poses(poses)
            if(len(poses) > 1):
                msg.position = poses
                pubposes.publish(msg)
                print "published"
            sleep(2)
        break
    file.close()
