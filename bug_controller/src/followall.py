#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
pub_ = None
regions_ = {
	'right': 0,
	'fright': 0,
	'front':0,
	'fleft':0,
	'left':0,
}
state_ = 0
state_dict_ = {
	0: 'find the wall',
	1: 'turn left',
	2: 'follow the wall',
}


def wall_follower_switch(req):
	global active_
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'done!'
	return res


def clbk_laser(msg):
	global regions_
	regions_ = {
		'right': min(min(msg.ranges[270:315]),10),
		'fright': min(min(msg.ranges[316:330]),10),
		'front': min(min(msg.ranges[0:30]),min(msg.ranges[331:360]),10),
		'fleft': min(min(msg.ranges[31:45]),10),
		'left': min(min(msg.ranges[46:90]),10),
	}

	take_action()

def change_state(state):
	global state_, state_dict_
	if state is not state_:
		print 'wall follower - [%s] - %s' %(state, state_dict_[state])
		state_ = state

def take_action():
	global regions_
	regions = regions_
	msg = Twist()
	linear_x = 0
	angular_z = 0
	check_dist = 0.40
	
	state_description = ''

	if regions['front'] > check_dist and regions['fleft'] > check_dist and regions['fright'] > check_dist: 
		state_description = 'case 1 - nothing'
		change_state(0)
	elif regions['front'] < check_dist and regions['fleft'] > check_dist and regions['fright'] > check_dist:
		state_description = 'case 2 - front'
		change_state(1)
	elif regions['front'] > check_dist and regions['fleft'] > check_dist and regions['fright'] < check_dist:
		state_description = 'case 3 - fright'
		change_state(2)
	elif regions['front'] > check_dist and regions['fleft'] < check_dist and regions['fright'] > check_dist:
		state_description = 'case 4 - fleft'
		change_state(0)
	elif regions['front'] < check_dist and regions['fleft'] > check_dist and regions['fright'] < check_dist:
		state_description = 'case 5 - front and fright'
		change_state(1)
	elif regions['front'] < check_dist and regions['fleft'] < check_dist and regions['fright'] > check_dist:
		state_description = 'case 6 - front and fleft'
		change_state(1)
	elif regions['front'] > check_dist and regions['fleft'] < check_dist and regions['fright'] < check_dist:
		state_description = 'case 7 - fleft and fright'
		change_state(0)
	elif regions['front'] < check_dist and regions['fleft'] < check_dist and regions['fright'] < check_dist:
		state_description = 'case 8 - front and fleft and fright'
		change_state(1)
	else:
		state_description = 'unknown case'
		rospy.loginfo(regions)

def find_wall():
	msg = Twist()
	vx = 0.15
	omega = math.pi/8.5
	msg.linear.x = vx
	msg.angular.z = -omega
	return msg

def turn_left():
	msg = Twist()
	omega = math.pi/5
	msg.angular.z = omega
	return msg

def follow_wall():
	global regions_
	vx_cruise = 0.25
	msg = Twist()
	msg.linear.x = vx_cruise
	return msg

def main():
	global pub_, active_

	rospy.init_node('follow_wall')

	pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

	srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if not active_:
			rate.sleep()
			continue

		msg = Twist()
		if state_ == 0:
			msg = find_wall()
		elif state_ == 1:
			msg = turn_left()
		elif state_ == 2:
			msg = follow_wall()
			pass
		else:
			rospy.logerr('unknown state!')

		pub_.publish(msg)

		rate.sleep()

if __name__ == '__main__':
	main()