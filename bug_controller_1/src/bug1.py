#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_precision_ = 5*(math.pi/180)
position_ = Point()
in_position_ = Point()
in_position_.x = rospy.get_param('initial_x')
in_position_.y = rospy.get_param('initial_y')
in_position_.z = 0
des_position_ = Point()
des_position_.x = rospy.get_param('des_pos_x')
des_position_.y = rospy.get_param('des_pos_y')
des_position_.z = 0
regions_ = None
state_desc_ = ['go to point', 'follow wall']
state_ = 0
count_state_time_ = 0
count_loop_ = 0
# 0 -> go to point
# 1 -> follow wall

def clbk_odom(msg):
	global position_, yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]

def clbk_laser(msg):
	global regions_
	regions_ = {
		'right': min(min(msg.ranges[270:315]),10),
		'fright': min(min(msg.ranges[316:330]),10),
		'front': min(min(msg.ranges[0:30]),min(msg.ranges[331:360]),10),
		'fleft': min(min(msg.ranges[31:45]),10),
		'left': min(min(msg.ranges[46:90]),10),
	}

def change_state(state):
	global state_, state_desc_
	global srv_client_wall_follower_, srv_client_go_to_point_
	global count_state_time_
	count_state_time_ = 0
	state_ = state
	log = "state changed: %s" % state_desc_[state]
	rospy.loginfo(log)
	if state_ == 0:
		resp = srv_client_go_to_point_(True)
		resp = srv_client_wall_follower_(False)
	if state_ == 1:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(True)

def distance_to_line(p0):
	# p0 is the current position
	# p1 and p2 define the line
	global in_position_, des_position_
	p1 = in_position_
	p2 = des_position_
	up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
	lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
	distance = up_eq / lo_eq

	return distance

def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def main():
	global regions_, position_, des_position_, state_, yaw_, yaw_precision_
	global srv_client_go_to_point_, srv_client_wall_follower_
	global count_state_time_, count_loop_
	
	rospy.init_node('bug1')
	
	sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	
	rospy.wait_for_service('/go_to_point_switch')
	rospy.wait_for_service('/wall_follower_switch')
	rospy.wait_for_service('/gazebo/set_model_state')
	
	srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
	srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
	srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	
	# set robot position
	model_state = ModelState()
	model_state.model_name = 'waffle'
	model_state.pose.position.x = in_position_.x
	model_state.pose.position.y = in_position_.y
	resp = srv_client_set_model_state(model_state)
	
	# initialize going to the point
	change_state(0)
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		
		if regions_ == None:
			continue
		
		distance_position_to_line = distance_to_line(position_)
		
		if state_ == 0:
			if regions_['front'] > 0.15 and regions_['front'] < 0.45:
				rospy.loginfo("front: [%.2f]", regions_['front'])
				change_state(1)
		
		elif state_ == 1:
			if distance_position_to_line < 0.1 and regions_['front'] > 0.45:
				rospy.loginfo("front: [%.2f]", regions_['front'])
				change_state(0)

		#count_state_time_ > 5 and \		
		#count_loop_ = count_loop_ + 1
		#if count_loop_ == 20:
		#	count_state_time_ = count_state_time_ + 1
		#	count_loop_ = 0
			
		rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
		rate.sleep()

if __name__ == "__main__":
	main()




