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
from geometry_msgs.msg import Twist

class follower():
    def __init__(self):
        #initialize the node
        rospy.init_node('follower',anonymous=False)
        rospy.on_shutdown(self.shutdown)
        #publisher to publish speed to turtlebot
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size = 10)
        #subscribing the coordinate information got from tracker node.
        #3 coordinates: x,y,radius of the circle detected
        self.coord_sub = rospy.Subscriber('/coord',String,self.coord_sub)
        self.tracking_coord = None
        #wait for some valid data to come in
        while self.tracking_coord is None:
            pass

        # TurtleBot will stop if we don't keep telling it to move. 
        # Therefore, we need set up the rate to 10 HZ.
        r = rospy.Rate(5)
        move_cmd = Twist()
        # unit for linear movement is m/s
        move_cmd.linear.x = 0 
        # unit for radius movement is rad/s
        move_cmd.angular.z = 0

        while not rospy.is_shutdown():
            if self.tracking_coord[0] is not None:
                #find out if turtlebot need turn
                diff_x = self.tracking_coord[0] - 320
		print(diff_x)
                if diff_x > 50:
                    angular_z_vel = -0.5
                if diff_x < -50:
                    angular_z_vel = 0.5
                if diff_x >=-50 and diff_x <=50:
                    angular_z_vel = 0
                #find out if turtlebot need forward or backward
                diff_z = self.tracking_coord[2] - 1000
                if diff_z > 50:
                    x_vel = 0.1
                elif diff_z <-50:
                    x_vel = -0.1
                else:
                    x_vel = 0
                print("Move foward vel {}, angular vel {}".format(x_vel,angular_z_vel))
		print("The location for turtlebot is {} along x-axis and {} along z-axis".format(diff_x,diff_z))
                move_cmd.linear.x = x_vel
                move_cmd.angular.z = angular_z_vel
                self.cmd_vel.publish(move_cmd)
            r.sleep()
    def shutdown(self):
        rospy.loginfo("stop turtlebot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def coord_sub(self,data):
	#print(data.data)
        coord = [float(x) for x in str(data.data).split(',')]
        self.tracking_coord = coord

def main(args):
  try:
      ic = follower()

  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
