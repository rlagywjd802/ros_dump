#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from math import sin,cos,atan2,sqrt,fabs,pi
from tf.transformations import euler_from_quaternion

class OneLink():
	def __init__(self):
		self.u1 = rospy.Publisher('/one_link/joint1_torque_controller/command', Float64, queue_size=1)
		self.q = rospy.Subscriber("/one_link/joint_states", JointState, self.callback)
		#self.q0 = rospy.Publisher("/one_link/joint_states", JointState, queue_size = 1)
		self.th1 = rospy.Publisher("/theta1", Float64, queue_size=1)
		self.rate = rospy.Rate(100) #10ms
		self.Kp = 5
		#initial = JointState()
		#initial.header.seq = 1
		#initial.header.stamp = rospy.Time.now()
		#initial.name = ['joint1']
		#initial.position = [45]
		#self.q0.publish(initial)
		#print("-----------------> initialized")

	def callback(self, joint_info):
		m1 = 1.0
		l1 = 0.5
		r1 = l1/2
		I1 = m1*r1*r1
		g = -9.806
		q1 = math.pi/2-(joint_info.position[0]+.1)
		qdot = joint_info.velocity[0]
		g1 = m1*g*r1*math.cos(q1)
		u = g1
		self.exert(u)
		print("q: "+str(q1*180/math.pi))
		print("u: "+str(u))
		self.rate.sleep()

	def exert(self, u):
		self.u1.publish(u)

	def stop(self):
		self.exert([0, 0])


if __name__ == '__main__':
	rospy.init_node('one_link_gravity_compensation', anonymous=False)
	my_robot = OneLink()
	try: 
		rospy.spin()
	except KeyboardInterrupt:
		my_robot.stop()
		print("Shutting down")
