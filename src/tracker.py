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

class tracker:

  def __init__(self):
    #self.image_pub = rospy.Publisher("/image_bn",Image,queue_size = 10)
    self.coord_pub = rospy.Publisher("coord",String, queue_size = 10)
    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.dep_cb)
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.tracked = [None,None,None]

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    gray_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray_image, 5)
    circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,1,170,param1=50,param2=30,minRadius=30,maxRadius=100)
    #circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,1,170,param1=50,param2=30,minRadius=0,maxRadius=100)

      
    circles = np.uint16(np.around(circles))
    for i in circles[0,0:1]:
                # if radius > 1 consider it as a ball,
                #if i[0]>100 and i[1] < 380 and i[1]>100 and i[0]<540:
                    #self.location[0] = i[0]
                    #self.location[1] = i[1]
                    cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)
		    location = "{},{},{}".format(i[0],i[1],i[2])
                    self.tracked[0] = i[0]
                    self.tracked[1] = i[1]
                    self.tracked[2] = i[2]
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    #print(self.tracked)
    #self.coord_pub.publish(location)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)
  def dep_cb(self,data):
    try:
      dep_image = self.bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)
    r2 = int(self.tracked[2])
    #mask = np.zeros((480,640))
    #mask[r2:-r2,r2:-r2] = 1
    #blur = cv2.medianBlur(dep_image, 7)
    #arr = np.array(blur,dtype=np.float32)
    #print(arr.shape)
    #arr = np.array(blur,dtype=np.uint8)
    arr = np.array(dep_image,dtype=np.float32)
    arr[0:r2,:] = 0
    arr[:,0:r2] = 0
    arr[-r2:,:] = 0
    arr[:,-r2:] = 0
    #rotate the image about y-axis
    #arr = cv2.flip(arr,1)
    if any(x is None for x in self.tracked[0:2]):
      print(self.tracked)
      #return
    else:
      #print(self.tracked)
      # to avoid the case when the center of the ball has depth nan, we take average depth of the ball
      #zz = []
      #r2 = int(self.tracked[2])
      #print(range(-int(r2),int(r2)))
      #for dx in range(-r2,r2):
        #print("dsad")
      #  for dy in range(-r2,r2):
      #    try:
      #      z = arr[self.tracked[0]+dx,self.tracked[1]+dy]
            #print(z)
      #    except:
      #      continue
      #    if not(np.isnan(z) or z == 0.0):
      #      zz.append(z)
      #print(len(zz))
      #if len(zz) >= 1:
      #  self.tracked[2] = np.mean(zz)
      #if arr[self.tracked[0],self.tracked[1]] >20:
      #  self.tracked[2] = arr[self.tracked[0],self.tracked[1]]
      #else:
      #  self.tracked[2] = 1000
      dep = np.mean(arr)
      location = '{},{},{}'.format(self.tracked[0],self.tracked[1],dep)
      #location = '{},{},{}'.format(self.tracked[0],self.tracked[1],self.tracked[2])
      print(location)
      self.coord_pub.publish(location)

def main(args):
  ic = tracker()
  rospy.init_node('tracker', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
