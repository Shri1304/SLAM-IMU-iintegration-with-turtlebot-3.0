#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publish_input():
    pub = rospy.Publisher('goal_input', String, queue_size=10)
    rospy.init_node('goal_input_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goal_str = raw_input("Enter a goal destination -> ");
        print("\nGoal set to " + goal_str)
        pub.publish(goal_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_input()
    except rospy.ROSInterruptException:
        pass