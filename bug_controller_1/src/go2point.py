#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

active_ = False
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
des_position_ = Point()
des_position_.x = rospy.get_param('des_pos_x')
des_position_.y = rospy.get_param('des_pos_y')
des_position_.z = 0
# parameters
yaw_precision_ = math.pi/90
dist_precision_ = 0.1
kP = 0.88

# publishers
pub = None

def go_to_point_switch(req):
	global active_
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'done!'
	return res

# callbacks
def clbk_odom(msg):
	global position_
	global yaw_

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

def change_state(state):
	global state_
	state_ = state
	print 'state changed to [%s]' % state_

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_heading(des_position_):
	global yaw_, pub, yaw_precision_, state_, kP
	des_yaw = math.atan2(des_position_.y - position_.y, des_position_.x - position_.x)
	#err_yaw = normalize_angle(des_yaw - yaw_)
	err_yaw = des_yaw - yaw_

	rospy.loginfo(err_yaw)

	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_:
		if err_yaw > 0:
			twist_msg.angular.z = kP*math.fabs(err_yaw)
		else:
			twist_msg.angular.z = -kP*math.fabs(err_yaw)

	pub.publish(twist_msg)

	if math.fabs(err_yaw) <= yaw_precision_:
		print 'yaw error: [%s]' % err_yaw
		change_state(1)

def go_straight_ahead(des_position_):
	global yaw_, pub, yaw_precision_, state_
	des_yaw = math.atan2(des_position_.y - position_.y, des_position_.x - position_.x)
	err_yaw = des_yaw - yaw_
	err_pos = math.sqrt(pow(des_position_.y - position_.y, 2) + pow(des_position_.x - position_.x, 2))

	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = 0.35
		pub.publish(twist_msg)
	else:
		print 'position error: [%s]' % err_pos
		change_state(2)

	if math.fabs(err_yaw) > yaw_precision_:
		print 'yaw error [%s]' % err_yaw
		change_state(0)

def done():
	twist_msg = Twist()
	twist_msg.linear.x = 0

	if math.fabs(yaw_) < 0.05:
		twist_msg.angular.z = 0
	else:
		twist_msg.angular.z = -yaw_

	pub.publish(twist_msg)

def main():
	global pub, active_

	rospy.init_node('go_to_point')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

	srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if not active_:
			continue
		else:
			if state_ == 0:
				fix_heading(des_position_)
			elif state_ == 1:
				go_straight_ahead(des_position_)
			elif state_ == 2:
				done()
			else:
				rospy.logger('unknown state!')

		rate.sleep()

if __name__ == '__main__':
	main()