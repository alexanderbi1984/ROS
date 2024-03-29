#! /usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
class follower:
    def __init__(self):
        #self.pub_ = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
	#rospy.init_node('follower',anonymous=True)
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher('/image_converter/output_video',Image,queue_size = 10)
        self.sub_image = rospy.Subscriber('/camera/rgb/image_raw', Image, self.img_cb)
        #self.sub_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.dep_cb)
        self.location = [None,None,None]
    def img_cb(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
            # Gaussian Blur if needed
            gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            blur = cv2.medianBlur(gray_img, 5)
            #circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,1,50,param1=50,param2=30,minRadius=0,maxRadius=50)
            #circles = np.uint16(np.around(circles))
            #for i in circles[0,:]:
                # if radius > 1 consider it as a ball
             #   if i[2]>1:
              #      self.location[0] = i[0]
               #     self.location[1] = i[1]
                #    cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                 #   cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
            cv2.imshow("detected ball",img)
            cv2.waitkey(3)
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img,"bgr8"))
    #def dep_cb(self,msg):
    #    try:
    #        img = self.bridge.imgmsg_to_cv2(msg)
    #    except CvBridgeError as e:
    #        print(e)
    #    if all(x is not None for x in self.location):
    #       print(x)
def main(args):
  fl = follower()
  rospy.init_node('follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

            


