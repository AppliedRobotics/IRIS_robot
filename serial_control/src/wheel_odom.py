#!/usr/bin/env python
# -*- coding: utf-8 -*-
#This script is made for testing messages using JointConf and JointCMD(see JointCmd example)
#Joint 0- left wheel
#Joint 1- right wheel
#Joint 2- head
#Structure of the message:
# (
#channel, P, I, D, rate(not used), 
#max_velocity(not used), lower_bound(not used), upper_bound(not used), limit_position(not used)
#)
#For wheels default settings: P=8, I=0.1, D=0;
#For Head: P=5, I= 0.75, D=0;
#Will work only with Firmware ver >= 0.8 
import rospy
import tf
from math import cos, sin, pi
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



rospy.init_node('talker07', anonymous=True)
pubodom = rospy.Publisher('odom', Odometry, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()



timeold =0
theta=0
x = y = 0
xold=yold=0
BASELINE= 0.5081
WHEELRADIUS= 0.115


##################joint_states###############
def jstatecb(jointstates):
 
    global timeold
    global x, y, theta,xold,yold
    xold=x
    yold=y
    ####odom part####

    time = rospy.Time.now()
    timenew= time.to_sec()
    delta = timenew -timeold
    if delta >=0.02:
        current_time = timenew
        timeold = timenew
        Vx =0 
        Vy = 0
        Vtheta = 0
        if jointstates.velocity[0] > 0.3 or jointstates.velocity[1] > 0.3:
            Vx, Vy, Vtheta=  calculate_odom(delta, jointstates.velocity[0], jointstates.velocity[1])
        #Sending to tf
        odom_quat = tf.transformations.quaternion_from_euler(0,0,theta)
        odom_broadcaster.sendTransform(
            (x,y,0.),
            odom_quat,
            time,
            "base_footprint",
            "odom"
        )
        laser_quat = tf.transformations.quaternion_from_euler(0,0,pi)
        odom_broadcaster.sendTransform((0.231,0,0.16), laser_quat, time, "laser", "base_footprint")
        camera_quat = tf.transformations.quaternion_from_euler(0,0,-pi/2)
        odom_broadcaster.sendTransform((0,0,0.5), camera_quat, time, "camera_rgb_optical_frame", "base_footprint") 
        map_quat = tf.transformations.quaternion_from_euler(0,0,0)
       # odom_broadcaster.sendTransform((0,0,0), map_quat, time, "odom", "map")
        
        #Sending to odometry
        odom = Odometry()
        odom.header.stamp = jointstates.header.stamp
        odom.header.frame_id = "odom"
        #set position
        odom.pose.pose = Pose(Point(x,y,0.), Quaternion(*odom_quat))
        #sev velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(Vx,0,0),Vector3(0,0,Vtheta))
        #publishing  
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[7] = 0.00001
        odom.twist.covariance[35] = 0.1
        pubodom.publish(odom)
        # if (xold!=x) & (yold!=y):
        	# print("x=",x," y=",y," theta=",theta)
    
    
    
##################odometry####################    
def calculate_odom(delta, Lvel, Rvel):
    global BASELINE
    global WHEELRADIUS
    global x, y, theta,xold,yold
    # Lvel= -Lvel 
    vel= (WHEELRADIUS)*(Rvel+Lvel)/2
    #to calc prev speed we need theta old(В локальной ск у нашего робота есть только 1 составляющая скорости)
    Vx= vel*sin(pi/2)
    Vy= vel*cos(pi/2)
    #New theta for caclulationg rotation matrix:
    Vtheta = (WHEELRADIUS)*(Rvel-Lvel)/BASELINE
    #Rotation matrix
    theta += delta * Vtheta
    x += delta* (cos(theta)*Vx - sin(theta)*Vy)
    y += delta* (sin(theta)*Vx + cos(theta)*Vy)
    return Vx, Vy, Vtheta
    
##################joystick####################


def listeners():
    global timeold
    timeold= rospy.get_time()
    rospy.Subscriber("joint_states_wheels", JointState, jstatecb)
    rospy.spin()
    rate =rospy.Rate(50)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
    
listeners()
