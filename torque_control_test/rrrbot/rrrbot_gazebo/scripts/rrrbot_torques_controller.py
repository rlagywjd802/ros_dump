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

class RRRBOT():
	def __init__(self):
		self.u1 = rospy.Publisher('/rrrbot/joint1_torque_controller/command', Float64, queue_size=1)
		self.u2 = rospy.Publisher('/rrrbot/joint2_torque_controller/command', Float64, queue_size=1)
		self.q = rospy.Subscriber("/rrrbot/joint_states", JointState, self.callback)
		self.th1 = rospy.Publisher("/theta1", Float64, queue_size=1)
		self.th2 = rospy.Publisher("/thata2", Float64, queue_size=1)
		self.rate = rospy.Rate(100) #10ms

		self.Kp = np.array([[5, 0], [0, 5]])

	def callback(self, joint_info):
		m1 = 1.0
		m2 = 1.0
		l1 = 0.5
		l2 = 0.5
		r1 = l1/2
		r2 = l2/2
		I1 = m1*r1*r1
		I2 = m2*r2*r2
		g = 9.806
		q = np.array([math.pi, 0])-(np.array(joint_info.position)+np.array([math.pi/2, 0]))
		#self.th1.publish(q[0]*180/math.pi)
		#self.th2.publish(q[1]*180/math.pi)
		qdot = np.array(joint_info.velocity)
		t1 = (m1*g*r1+m2*g*l1)*math.cos(q[0]) + m2*g*r2*math.cos(q[0]+q[1])
		t2 = m2*g*r2*math.cos(q[0]+q[1])
		G = np.array([t1, t2])
		#u = np.array([0, 0])		
		u = -G + 4*qdot
		#u = -G
		self.exert(u)
		#print("q: "+str(q*180/math.pi))
		print("u: "+str(u))
		self.rate.sleep()

	def exert(self, u):
		self.u1.publish(u[0])
		self.u2.publish(u[1])
		self.th1.publish(u[0])
		self.th2.publish(u[1])

	def stop(self):
		self.exert([0, 0])


if __name__ == '__main__':
	rospy.init_node('rrrbot_gravity_compensation', anonymous=True)
	my_robot = RRRBOT()
	try: 
		rospy.spin()
	except KeyboardInterrupt:
		my_robot.stop()
		print("Shutting down")
