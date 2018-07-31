#! /usr/bin/env python
import rospy
from robot_state_publisher.joint_state_listener import Int16

def messageCb(msg_data):
	rospy.loginfo("receive digit = %s", msg_data.data)

def listner():
	rospy.init_node("digit_tracker", anonymous=False)
	rospy.Subscriber("mnist_digit", Int16, messageCb)
	rospy.spin()

if __name__ == '__main__':
	listner()
