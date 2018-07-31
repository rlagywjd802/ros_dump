#! /usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64

def talker():
	rospy.init_node('cubic_spline_task', anonymous=True)
	cubic_pub = rospy.Publisher('/excavator/joint1_position_controller/command', Float64, queue_size=100)
	r = rospy.Rate(100)
	i = 0
	
	while not rospy.is_shutdown():
		q = math.sin(i/100.0)
		rospy.loginfo("des_q = %f" %q)
		cubic_pub.publish(q)
		i = i+1
		r.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
