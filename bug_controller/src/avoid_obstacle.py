#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi

pub = None

def clbk_laser(msg):
	regions = {
		'right': min(min(msg.ranges[270:315]),10),
		'fright': min(min(msg.ranges[316:330]),10),
		'front': min(min(msg.ranges[0:30]),min(msg.ranges[331:360]),10),
		'fleft': min(min(msg.ranges[31:45]),10),
		'left': min(min(msg.ranges[46:90]),10),
	}

	take_action(regions)

def take_action(regions):
	msg = Twist()
	linear_x = 0
	angular_z = 0
	check_dist = 0.35
	vx = 0.25
	omega = pi/10

	state_description = ''

	if regions['front'] > check_dist and regions['fleft'] > check_dist and regions['fright'] > check_dist: 
		state_description = 'case 1 - nothing'
		linear_x = vx
		angular_z = 0
	elif regions['front'] < check_dist and regions['fleft'] > check_dist and regions['fright'] > check_dist:
		state_description = 'case 2 - front'
		linear_x = 0
		angular_z = omega
	elif regions['front'] > check_dist and regions['fleft'] > check_dist and regions['fright'] < check_dist:
		state_description = 'case 3 - fright'
		linear_x = 0
		angular_z = omega
	elif regions['front'] > check_dist and regions['fleft'] < check_dist and regions['fright'] > check_dist:
		state_description = 'case 4 - fleft'
		linear_x = 0
		angular_z = -omega
	elif regions['front'] < check_dist and regions['fleft'] > check_dist and regions['fright'] < check_dist:
		state_description = 'case 5 - front and fright'
		linear_x = 0
		angular_z = omega
	elif regions['front'] < check_dist and regions['fleft'] < check_dist and regions['fright'] > check_dist:
		state_description = 'case 6 - front and fleft'
		linear_x = 0
		angular_z = -omega
	elif regions['front'] > check_dist and regions['fleft'] < check_dist and regions['fright'] < check_dist:
		state_description = 'case 7 - fleft and fright'
		linear_x = -vx
		angular_z = 0
	elif regions['front'] < check_dist and regions['fleft'] < check_dist and regions['fright'] < check_dist:
		state_description = 'case 8 - front and fleft and fright'
		linear_x = 0
		angular_z = omega
	else:
		state_description = 'unknown case'
		rospy.loginfo(regions)

	rospy.loginfo(state_description)
	msg.linear.x = linear_x
	msg.angular.z = angular_z
	pub.publish(msg)

def main():
	global pub

	rospy.init_node('reading_laser')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

	rospy.spin()

if __name__ == '__main__':
	main()