#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.srv import GetMap, GetPlan
from std_msgs.msg import Bool
import tf
from math import pi, cos, sin, sqrt, atan2
import numpy as np

class Explorer:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.psoe_sub = rospy.Subscriber("target_pose", Point, self.pose_cb)
        self.status_pub = rospy.Publisher("pose_status", Bool, queue_size = 5)
        self.target_pose = (0,0,0)

    def pose_cb(self,data):
        self.target_pose = (data.x, data.y, data.z)
        self.explore()

    def explore(self):
        print(self.target_pose)
        status = self.reach_goal(self.target_pose)
        status_msg = Bool()
        print(status)
        print(type(status))
        if(status == True or status == 3):
            print"I successfuly reached goal"
            status_msg.data = True
        else:
            print("Move base failed with status : {}".format(status))
            status_msg.data = False
        # status_msg.data = True
        # self.client.cancel_goal()
        rospy.sleep(0.1)
        self.status_pub.publish(status_msg)

    def reach_goal(self, goal_pose):
        x, y, theta = goal_pose
        target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        t0=rospy.Time.now()
        goal=MoveBaseGoal()
        goal.target_pose.header.stamp=t0
        goal.target_pose.header.frame_id="map"

        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation.x = target_quat[0]
        goal.target_pose.pose.orientation.y = target_quat[1]
        goal.target_pose.pose.orientation.z = target_quat[2]
        goal.target_pose.pose.orientation.w = target_quat[3]  

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        # let us wait 1 minute to reach the goal and abort otherwise
        reached_the_goal = self.client.wait_for_result(rospy.Duration.from_sec(90))
        if not reached_the_goal:
            rospy.logwarn("I was not able to reach the goal within the\
                          allocated time")
            self.client.cancel_goal()
            
        # Prints out the result of executing the action
        return self.client.get_state()


if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    explorer = Explorer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

        