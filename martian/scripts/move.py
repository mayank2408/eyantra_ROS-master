#!/usr/bin/env python

""" move_base_square.py - Version 1.1 2013-12-20
    Command a robot to move in a square using move_base actions..
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""
import time
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi
from visualization_msgs.msg import Marker

#to hold the most recently detected aruco terrain value
terrain=''
last_visited=(0,0)
last_point=Point(0,0,0)
arucoID,sr_no=0,0
#Dictionary for mapping the aruco marker values to their respective terrains
env={200:'Sandy',201:'Clay',202:'Silty sand',203:'Rain Water',204:'Ice',205:'Snow water',206:'Sedimentary', 207:'Metamorphic', 208:'Igneous', 209:'Oxides', 210:'Carbonates',211:'Phosphates',212:'Sulfides', 213:'Native Element', 214:'Silicates',217:'OBSTACLE'}


#The list of Site of interests
a={ 	(3,0): (203, 'y'), 
	(6,2): (209, 'y'),
	(0,3): (214, 'x'), 
	(4,3): (201, 'x'),  
	(2,5): (211, 'x'), 
	(6,5): (217, 'y')}
	
d={ 	203 : (3,0),
	209 : (6,2),
	214 : (0,3),
	201 : (4,3),
	211 : (2,5),
	217 : (6,5)}
#callback msg for Aruco marker detection
done=[]
def callback_marker(request):
	global env,terrain,arucoID,done
	k=request.id
	if k in env.keys(): 
		terrain=env[k]
		arucoID=k
		#rospy.loginfo(terrain)
	if (arucoID not in done) :
		chkAruco()
		done.append(arucoID)
	'''	
	else :
		terrain='OBSTACLE'
		arucoID=-1
		#rospy.loginfo('no record found of this marker')
	'''	
def chkAruco():
	global terrain,arucoID,a,d,last_visited,sr_no
	if arucoID in d.keys():				 
		rospy.loginfo(str(sr_no)+':'+str(last_visited)+':'+str(d[arucoID])+':'+str(arucoID)+':'+terrain)
		sr_no+=1
		last_visited=d[arucoID]
	else:
		rospy.loginfo('Something went wrong')
#Aruco marker subscrriber
rospy.Subscriber('Estimated_marker',Marker,callback_marker)


class MoveBaseSquare():
	def __init__(self):
		global a
		rospy.init_node('nav_test', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		soi_len=len(a)
		
		#list to hold the soi points converted from feet to points rviz map
		soi_points=list()
		#list to hold the euler angles for each soi point
		euler_angles = list()
		quaternions = {}
		#conversion of soi point to point in rviz map
		for soi in a:
			eul=0
			p=Point(0,0,0)
			p.x=soi[0]*0.3048
			p.y=soi[1]*0.3048
			if a[soi][1]=='x':
				eul=pi
				p.x+=(0.3048/1.35)
				p.y-=(0.3048/2)
				#if chkid==1: p.x
			else:
				eul=pi/2
				p.y+=(0.3048/1.3)
				p.x-=(0.3048/2)
			soi_points.append(p)
			#euler_angles.append(eul)
			quaternions[p]=eul
        
		# Create a list to hold the target quaternions (orientations)
		
		# Then convert the angles to quaternions
		'''for angle in euler_angles:
			q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
			q = Quaternion(*q_angle)
			quaternions.append(q)'''
		
		#Code to roughly calculate a relatively shorter path to traverse all nodes
		
		#initialize list to hold final co-ordinates and angles 
		f_soi_list=list()
		f_q_list=list()
		last=Point(0,0,0)
		while len(soi_points)>0:
			curr_point=-1
			curr_short=1000000000.0
			for point in range(len(soi_points)):
				#find the closest point to the current point
				dist=pow((pow((soi_points[point].x-last.x),2)+pow((soi_points[point].y-last.y),2)),0.5)
				if dist < curr_short:
					curr_point=point
					curr_short=dist
			last=soi_points[curr_point]
			angle=quaternions[last]
			q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
			q = Quaternion(*q_angle)
			f_q_list.append(q)
			f_soi_list.append(last)
			soi_points.remove(last)
			#euler_angles.remove(angle)
		
		# Create a list to hold the waypoint poses
		waypoints = list()
		
		# give waypoints in the format :  waypoints.append(Pose(Point(1.66,1.66,-0.00143), quaternions[0]))
		
		curr_pos=Point(0,0,0)
		for n in range(soi_len):
			next_pos=f_soi_list[n]
			curr_point=(int(curr_pos.x/0.3048),int(curr_pos.y/0.3048))
			q_angle=-1
			
			if curr_pos.x<=next_pos.x and curr_pos.y<=next_pos.y :
				temp=(curr_point[0]+1,curr_point[1])
				temp2=(curr_point[0],curr_point[1]+1)
				if (temp not in a.keys()) and temp[0]>=0 and temp[1]>=0:
					q_angle=0
				elif (temp2 not in a.keys()) and temp2[0]>=0 and temp2[1]>=0:
					q_angle=pi/2
			elif curr_pos.x>next_pos.x and curr_pos.y<=next_pos.y :
				temp=(curr_point[0]-1,curr_point[1])
				temp2=(curr_point[0],curr_point[1]+1)
				if temp2 not in a.keys() and temp[0]>=0 and temp[1]>=0:
					q_angle=pi/2
				elif temp not in a.keys() and temp2[0]>=0 and temp2[1]>=0:
					q_angle=pi
			elif curr_pos.x<=next_pos.x and curr_pos.y>next_pos.y :
				temp=(curr_point[0]+1,curr_point[1])
				temp2=(curr_point[0],curr_point[1]-1)
				if temp2 not in a.keys() and temp[0]>=0 and temp[1]>=0:
					q_angle=((3*pi)/2)
				elif temp not in a.keys() and temp2[0]>=0 and temp2[1]>=0:
					q_angle=0
			else:
				temp=(curr_point[0]-1,curr_point[1])
				temp2=(curr_point[0],curr_point[1]-1)
				if temp not in a.keys() and temp[0]>=0 and temp[1]>=0:
					q_angle=pi
				elif temp2 not in a.keys() and temp2[0]>=0 and temp2[1]>=0:
					q_angle=((3*pi)/2)
			if (not(q_angle==-1) and not((curr_point[0]+curr_point[1])==0)):
				q = Quaternion(*quaternion_from_euler(0, 0, q_angle, axes='sxyz'))
				waypoints.append(Pose(curr_pos,q)) 
				#print curr_point
				#print q_angle
			waypoints.append(Pose(f_soi_list[n],f_q_list[n]))
			curr_pos=f_soi_list[n]

		# Publisher to manually control the robot (e.g. to stop it, queue_size=5)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

		# Subscribe to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)


		# Wait 60 seconds for the action server to become available
		self.move_base.wait_for_server(rospy.Duration(60))
		
		#rospy.loginfo("Connected to move base server")
		#rospy.loginfo("Starting navigation test")
		
		# Initialize a counter to track waypoints
		i = 0
		soi_len=len(waypoints)
		# Cycle through the six waypoints
		while i < soi_len and not rospy.is_shutdown():
			
			# Intialize the waypoint goal
			goal = MoveBaseGoal()
			
			# Use the map frame to define goal poses
			goal.target_pose.header.frame_id = 'map'
			
			# Set the time stamp to "now"
			goal.target_pose.header.stamp = rospy.Time.now()
			
			# Set the goal pose to the i-th waypoint
			goal.target_pose.pose = waypoints[i]
			#print waypoints[i].position
			#print waypoints[i].orientation
			
			# Start the robot moving toward the goal
			self.move(goal,waypoints[i])
			i += 1
		rospy.loginfo("MISSION ACCOMPLISHED")

	def move(self, goal,waypoint):
		global last_point
		# Send the goal pose to the MoveBaseAction server
		self.move_base.send_goal(goal)

		# Allow 1 minute to get there
		finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
            
		# If we don't get there in time, abort the goal
		if not finished_within_time:
			self.move_base.cancel_goal()
			#rospy.loginfo("Timed out achieving goal")
		#else:
		#	# We made it!
		#	state = self.move_base.get_state()
		#	if state == GoalStatus.SUCCEEDED:
		#		rospy.loginfo("Goal succeeded!")
		time.sleep(3)
		'''		
		if not(last_point==waypoint.position):
			time.sleep(7);
			self.chkAruco()
		last_point=waypoint.position
		'''
		
		
	def shutdown(self):
		#rospy.loginfo("Stopping the robot...")
		# Cancel any active goals
		self.move_base.cancel_goal()
		rospy.sleep(2)
		# Stop the robot
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':
    MoveBaseSquare()
    #try:
    #    MoveBaseSquare()
    #except rospy.ROSInterruptException:
	#rospy.loginfo("Navigation test finished.")
