#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_bn",Image,queue_size = 10)
    self.coord_pub = rospy.Publisher("coord",String, queue_size = 10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    gray_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray_image, 5)
    circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,1,170,param1=50,param2=30,minRadius=40,maxRadius=100)
    circles = np.uint16(np.around(circles))
    for i in circles[0,0:1]:
                # if radius > 1 consider it as a ball
                if i[2]>1:
                    #self.location[0] = i[0]
                    #self.location[1] = i[1]
                    cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)
		    location = "{},{},{}".format(i[0],i[1],i[2])
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    self.coord_pub.publish(location)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
