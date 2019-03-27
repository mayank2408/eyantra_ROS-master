#!/usr/bin/env python

'''
Team Id : eYRC-EB#3848
Author List : Mayank Gupta
Filename : odometry_calc.py
Theme : Explorer Bot (Track 2)
Functions : callback_left(), callback_right(), callback_vel(), callback_yaw()
Global Variables : current_time1, last_time1, DistancePerCount, current_time, last_time, l_dis, r_dis, avg_dis, vx, vy, vth, x, y, th, tick_x, tick_y
'''

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
 
LeftEncoderCounts=0
RightEncoderCounts=0
#variable for yaw
yo=0

rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()					#odom transform for movement rviz

#variables to be used in calculations
current_time1 = rospy.Time.now()				
last_time1 = rospy.Time.now()
DistancePerCount = (0.204203522) / 1680.0
current_time = rospy.Time.now()
last_time = rospy.Time.now()
l_dis = 0
r_dis = 0
avg_dis=0
vx=0.0
vy=0.0
vth=0.0
x = 0.0
y = 0.0
th = 0.0
tick_x=0.0
tick_y=0.0



def callback_left(ticks):
        global LeftEncoderCounts
        LeftEncoderCounts=ticks.data

def callback_right(ticks):
        global RightEncoderCounts
        RightEncoderCounts=ticks.data

def callback_yaw(angle):
        global yo
        yo=angle.data

def callback_vel(vel):
        global vx,vy,vth
	vx=vel.linear.x
        vy=vel.linear.y
        vth=vel.angular.z

rospy.Subscriber("right_enc", Int32	, callback_right)				#Right Motor Encoder data subscriber
rospy.Subscriber("left_enc", Int32, callback_left)				#Left Motor Encoder data subscriber
rospy.Subscriber("cmd_vel", Twist, callback_vel)					#Command velocity from Base_local_planner subscriber for robot speed
rospy.Subscriber("yaw", Float32, callback_yaw)						#Robot angle subscriber from IMU unit on the robot

r = rospy.Rate(70.0)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    
    #if we want to do localization of the robot ourselves using encoder data
    
    deltaLeft = LeftEncoderCounts - tick_x
    deltaRight =RightEncoderCounts - tick_y
    l_dis = (deltaLeft * DistancePerCount)
    r_dis = (deltaRight * DistancePerCount)
    avg_dis=(r_dis+l_dis)/2
    tick_x=LeftEncoderCounts
    tick_y=RightEncoderCounts
    # compute odometry in a typical way given the velocities of the robot
    x += avg_dis*cos(th)
    y += avg_dis*sin(th)
    th = yo
    
    '''
    
    #Calculate the movement of the robot from its current position according to the given velocities from move.py
    dt = (current_time - last_time).to_sec();		
    delta_x = (vx * cos(th)) * dt;
    delta_y = (vx * sin(th)) * dt;
    delta_th = vth * dt;
    

    #the new position of the robot
    x += delta_x;
    y += delta_y;
    th += delta_th;
    '''
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"	

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
