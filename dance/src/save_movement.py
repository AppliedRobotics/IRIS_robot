#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios



msg = """
setup your dance
press s to save position
"""

e = """
Communications Failed
"""
positions = []
def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def jstatecb(data):
    global positions
    positions = data.position

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('init_dance')
    rospy.Subscriber("arm_joint_states",JointState,jstatecb)
    file = open('/home/iris/iris_ws/src/dance/src/movements.txt', 'w')
    try:
        while(1):
            key = getKey()
            if key == 's' :
		print positions
                for pose in positions:
                    file.write(str(pose)+',')
                file.write('\n')
                print("your pose saved")
            elif (key == '\x03'):
                break
    except:
        file.close()
        print e

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
