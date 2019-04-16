#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import rospy
import time
import math

wait_loop = True
pose = PoseStamped()
pose.header.frame_id = 'map'
pose.pose.position.z = 0

def publish_pose(imu_data):
	# Publish the imu data as a pose in RVIZ
	global pose
	global pose_pub

	data = [0, 0, 0, 0, 0]
	data[0] = imu_data.linear_acceleration.x
	data[1] = imu_data.linear_acceleration.y
	data[2] = imu_data.orientation.w
	data[3] = imu_data.header.stamp.secs #timestamp in seconds
	data[4] = imu_data.header.stamp.nsecs #nanosec since time stamp

	x, y, w = process_imu(data)

	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.orientation.z = w

	pose_pub.publish(pose)


def process_imu(data):

	# TIME ISSUE ... EITHER WAIT LONGER AT START OR USE PYTHON TIME

	# Process the imu data by acquiring the yaw and integrating
	# to get x and y position
	global wait_loop
	global time_prev
	global acc_prev
	global vel_prev
	global vel
	global pos
	global time_passed
	global time_now
	global vel_sum
	global pos_sum

	if wait_loop:
		vel = 0
		vel_prev = [0, 0]
		pos = [0, 0]
		acc_prev = [0, 0]
		time_passed = 0
		time_prev = data[3] + (data[4]/1e9)
		#time_prev = time.time()
		time_now = 0
		vel_sum = [0, 0]
		pos_sum = [8, -1]
		wait_loop = False
		return [0, 0, 0]

	# Calculate time passed since last message
	time_now = data[3] + (data[4]/1e9)
	#time_now = time.time()
	time_passed = time_now - time_prev
	time_prev = time_now

	# Yaw from yaw sensor
	w = math.degrees(data[2]) + 180

	# Read in acceleration and integrate using trapezoid
	acc = [data[0]*4, data[1]*1.3]
	for i, acc_val in enumerate(acc):
		vel = 0.5 * time_passed * (acc_val + acc_prev[i])
		vel_sum[i] = vel_sum[i] + vel
		acc_prev[i] = acc_val

		pos = 0.5 * time_passed * (vel + vel_prev[i])
		if i == 1:
			pos = -1*pos

		pos_sum[i] = pos_sum[i] + pos
		vel_prev[i] = vel

	return pos_sum[0], pos_sum[1], w

def discrete_highpass(x, x_prev, y_prev, fc, time_passed):
	alpha = 1 / ((2*pi*time_passed*fc) + 1)
	y = (alpha*y_prev) + (alpha*(x - x_prev))
	y_prev = y
	return y, y_prev

def discrete_lowpass(x, y_prev, fc, time_passed):
	term1 = 2*pi*time_passed*fc
	alpha = term1/(term1 + 1)
	y = (alpha*x) + ((1-alpha)*y_prev)
	y_prev = y
	return y, y_prev


def pose_publisher():
	# Setup the publisher node
	global pose_pub
	pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=5)
	rospy.init_node('pose_publisher', anonymous=True)
	rate = rospy.Rate(10)

	rospy.Subscriber('/imu', Imu, publish_pose)
	rospy.spin()
		

if __name__ == '__main__':
	try:
		pose_publisher()
	except rospy.ROSInterruptException:
		pass
