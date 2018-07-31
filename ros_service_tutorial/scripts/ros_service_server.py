#! /usr/bin/env python
import rospy
import math
import numpy as np
from ros_service_tutorial.srv import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class InverseKinematics():
	def __init__(self):
		self.two_link_server = rospy.Service('ros_service', TutorialService, self.callback)
		self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
		self.r = rospy.Rate(100)

	def callback(self, req):
		if req.digit == 0:
			res = True
			str2 = "Service result : %s" %(res)
			rospy.loginfo(str2)
			self.talker()
		else:
			res = False
		return TutorialServiceResponse(res)

	def talker(self):		
		i = 0
		markerArray = MarkerArray()
		while not rospy.is_shutdown():
			xd = np.array([0.1*math.sin(i/100.0), 0.1*math.cos(i/100.0)+0.5])
			if i%10 == 0:
				marker = Marker()
				marker.header.frame_id = "/map"
				marker.type = marker.SPHERE
				marker.action = marker.ADD
				marker.pose.position.x = xd[0]
				marker.pose.position.y = xd[1]
				marker.pose.position.z = 1
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
			i = i+1
			self.r.sleep()

def main():
	rospy.init_node('ros_service_server', anonymous=False)
	ik = InverseKinematics()
	try:		
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	rospy.loginfo("Service server is ready")

if __name__ == '__main__':
	main()