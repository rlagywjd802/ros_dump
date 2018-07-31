#! /usr/bin/env python
import sys, os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from lib.mnist import load_mnist
from lib.network import TwoLayerNet
from mnist_publisher.srv import *
import time

package_path = "/home/hyojeong/catkin_ws/src/mnist_publisher/"

network = TwoLayerNet(input_size=784, hidden_size=50, output_size=10)
network.restore_network(package_path)

class image_converter():

    def __init__(self):
        # self.network = TwoLayerNet(input_size=784, hidden_size=50, output_size=10)
        # self.network.restore_network(package_path)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/digit_image", Image, self.callback)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size = 1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv_image_flatten = cv_image.reshape(-1, 784)
            with network.session.as_default():
                digit_result = np.argmax(network.predict(cv_image_flatten))
                rospy.loginfo("I think it is "+ str(digit_result))
                rospy.loginfo("Receive Service, Response Result : " + str(self.service_client(digit_result)))
        except CvBridgeError as e:
            print(e)
        
        ret,thresh = cv2.threshold(cv_image,200,255,0)
        cv_image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        cv2.drawContours(cv_image_color, contours, 0, (0, 255, 0), 1)
        try:
          #self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh, encoding="mono8"))
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image_color, encoding="bgr8"))
        except CvBridgeError as e:
          print(e)

    def service_client(self, x):
        rospy.wait_for_service('move_command')
        try:
            ros_service = rospy.ServiceProxy('move_command', TwoLinkService)
            res = ros_service(x)
            return res.end
        except rospy.ServiceException, e:
            print("Failed to call ros_service_server")

def main(args):
    # (x_test, t_test), (x_train, t_train) = load_mnist(path=package_path+'dataset')    
    rospy.init_node('image_converter', anonymous=True)
    
    ic = image_converter()    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
