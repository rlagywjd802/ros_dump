#! /usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState

class JointPublisher():
	def __init__(self):
		self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
		self.r = rospy.Rate(10)

	def loop(self):
		i = 0
		while not rospy.is_shutdown():
			q1 = math.sin(i/100)
			self.joint_publish(q1)
			i += 1

	def joint_publish(self, q):
		joint_command = JointState()
		joint_command.header.stamp = rospy.Time.now()
		joint_command.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		joint_command.position = [q, 0, 0, 0, 0, 0]
		self.joint_pub.publish(joint_command)


if __name__ == '__main__':	
	rospy.init_node('ur_joint_publisher', anonymous=True)
	ur = JointPublisher()
	ur.loop()