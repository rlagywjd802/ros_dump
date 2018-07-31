#! /usr/bin/env python
import rospy
import math
import sys
import numpy as np
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from two_link_ik.srv import *

MARKERS_MAX = 20

class InverseKinematics():

	def __init__(self):
		self.two_link_server = rospy.Service('move_command', TwoLinkService, self.callback)
		self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
		self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
		self.dt = 0.01
		self.q0 = [0.93, 1.29]
		self.Kp = 5
		self.l1 = 0.5
		self.l2 = 0.5
		self.r = rospy.Rate(1/self.dt)

	def callback(self, req):
		if req.digit == 0:
			res = True
			self.talker()
		else:
			res = False
		return TwoLinkServiceResponse(res)

	def talker(self):		
		i = 0
		q = self.q0
		markerArray = MarkerArray()
		while not rospy.is_shutdown() and i<100*2*math.pi:
			# xd: desired x, y
			# x : current x, y
			# e : tracking error
			xd = np.array([0.2*math.sin(i/100.0), 0.2*math.cos(i/100.0)+0.6])
			x = self.forward_kinematics(q)
			e = xd-x
			x_dot = self.Kp*e
			q_dot = np.dot(self.jacobian_inverse(q), x_dot)
			q = q + q_dot*self.dt
			self.joint_publish(i, q)
			if i%10 == 0:
				markerArray = self.marker_publish(xd, markerArray)
			i = i+1
			self.r.sleep()


	def joint_publish(self, i, q):
		joint_command = JointState()
		joint_command.header.seq = i		
		joint_command.header.stamp = rospy.Time.now()
		joint_command.name = ['joint1', 'joint2']
		joint_command.position = q
		self.joint_pub.publish(joint_command)

	
	def marker_publish(self, x, markerArray):
		marker = Marker()
		marker.header.frame_id = "/base"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.pose.position.x = x[0]
		marker.pose.position.y = x[1]
		marker.pose.position.z = 0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		markerArray.markers.append(marker)
		id = 0
		for m in markerArray.markers:
			m.id = id
			id += 1
		self.marker_pub.publish(markerArray)
		return markerArray


	def forward_kinematics(self, q):
		x = self.l1*math.cos(q[0]) + self.l2*math.cos(q[0]+q[1])
		y = self.l1*math.sin(q[0]) + self.l2*math.sin(q[0]+q[1])
		return np.array([x, y])


	def jacobian_inverse(self, q):
		J = np.array([[-self.l1*math.sin(q[0]), -self.l2*math.sin(q[0]+q[1])],
					[self.l1*math.cos(q[0]), self.l2*math.cos(q[0]+q[1])]])
		J_inv = J.transpose()
		return J_inv


def main(args):
	rospy.init_node('two_link_ik', anonymous=False)	
	ik = InverseKinematics()
	try:		
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':	
	main(sys.argv)