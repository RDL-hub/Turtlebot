#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_precision_ = 5*(math.pi/180)
position_ = Point()
desired_position_ = Point()
#desired_position_.x = rospy.get_param('des_pos_x')
#desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.x = 5
desired_position_.y = -1
desired_position_.z = 0
regions_ = None
state_desc_ = ['go to point', 'follow wall']
state_ = 0
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
	state_ = state
	log = "state changed: %s" % state_desc_[state]
	rospy.loginfo(log)
	if state_ == 0:
		resp = srv_client_go_to_point_(True)
		resp = srv_client_wall_follower_(False)
	if state_ == 1:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(True)

def normalize_angle(angle):
	if math.fabs(angle) > math.pi:
		angle = angle - (2*math.pi*angle)/(math.fabs(angle))
	return angle

def main():
	global regions_, position_, desired_position_, state_, yaw_, yaw_precision_
	global srv_client_wall_follower_, srv_client_go_to_point_

	rospy.init_node('bug0')

	sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

	rospy.wait_for_service('/go_to_point_switch')
	rospy.wait_for_service('/wall_follower_switch')

	srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
	srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)

	change_state(0)

	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		if regions_ == None:
			continue

		if state_ == 0:
			desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
			#rospy.loginfo("error on the angle: [%.2f]", (desired_yaw - yaw_ - math.pi/2))
			rospy.loginfo("actual angle: [%.2f]", yaw_)
			rospy.loginfo("desired angle: [%.2f]", desired_yaw)
			if regions_['front'] > 0.15 and regions_['front'] < 0.50:
				change_state(1)

		elif state_ == 1:
			desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
			err_yaw = normalize_angle(desired_yaw - yaw_)
			
			if err_yaw > 0 and \
				math.fabs(err_yaw) > (math.pi/4) and \
				regions_['front'] > 0.50:
					change_state(0)

			if err_yaw > 0 and \
				math.fabs(err_yaw) > (math.pi/4) and \
				math.fabs(err_yaw) < (math.pi/2) and \
				regions_['left'] > 0.50:
					change_state(0)

			if err_yaw < 0 and \
				math.fabs(err_yaw) > (math.pi/4) and \
				math.fabs(err_yaw) < (math.pi/2) and \
				regions_['right'] > 0.50:
					change_state(0)

		rate.sleep()

if __name__ == '__main__':
	main()