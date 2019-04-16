#!/usr/bin/env python

from visualization_msgs.msg import Marker
from std_msgs.msg import String
import rospy

marker = Marker()
marker.header.frame_id = "/map"
marker.type = marker.SPHERE
marker.action = marker.ADD
marker.scale.x = 0.8
marker.scale.y = 0.8
marker.scale.z = 0.1
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 0.5
marker.color.b = 0.8
marker.pose.orientation.w = 1.0
marker.pose.position.z = 0


def publish_goal(data):
	global marker
	global marker_pub

	goal = data.data;

	if goal == 'kitchen':
		x = 0
		y = 0
	elif goal == 'study space':
		x = 10
		y = -10
	elif goal == 'bathroom':
		x = 9
		y = -3
	elif goal == 'water fountain':
		x = 11
		y = -6

	marker.pose.position.x = x
	marker.pose.position.y = y

	marker_pub.publish(marker)

def goal_updater():
	rospy.init_node('goal_updater', anonymous=True)
	global marker_pub
	marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
	rate = rospy.Rate(10)

	rospy.Subscriber('/goal_input', String, publish_goal)
	rospy.spin()

if __name__ == '__main__':
	try:
		goal_updater()
	except rospy.ROSInterruptException:
		pass
