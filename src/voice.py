#!/usr/bin/env python

"""This module is a simple demonstration of voice control
for ROS turtlebot using pocketsphinx
"""

#import argparse
import roslib
import rospy
from os import path
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist,Pose,Point,Quaternion
from tf.transformations import quaternion_from_euler

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
from sound_play.libsoundplay import SoundClient


class ASRControl(object):
    """Simple voice control interface for ROS turtlebot

    Attributes:
        model: model path
        lexicon: pronunciation dictionary
        kwlist: keyword list file
        pub: where to send commands (default: 'mobile_base/commands/velocity')

    """
    def __init__(self):
        # initialize ROS
        self.speed = 0.1
        self.msg = Twist()

        rospy.init_node('voice_cmd')
        rospy.on_shutdown(self.shutdown)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.soundhandle = SoundClient()

        rospy.loginfo("Waiting for move_base action server")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")

        # you may need to change publisher destination depending on what you run
        pub = 'mobile_base/commands/velocity'
        self.pub_ = rospy.Publisher(pub, Twist, queue_size=10)
        # setting up the necessary parameters for sphinx
        model = '/usr/local/lib/python2.7/dist-packages/pocketsphinx/model/en-us'
        lexicon = '/home/nbi1/catkin_ws/src/BiN/src/voice_cmd.dic'
        kwlist = '/home/nbi1/catkin_ws/src/BiN/src/voice_cmd.kwlist'
        # initialize pocketsphinx
        config = Decoder.default_config()
        config.set_string('-hmm', model)
        config.set_string('-dict', lexicon)
        config.set_string('-kws', kwlist)

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()

        self.decoder = Decoder(config)
        self.decoder.start_utt()

        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            self.parse_asr_result()
            
        

    def parse_asr_result(self):
        """
        move the robot based on ASR hypothesis
        """
        if self.decoder.hyp() != None:
            print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in self.decoder.seg()])
            print ("Detected keyphrase, restarting search")
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()
            goal = MoveBaseGoal()

            # you may want to modify the main logic here
            if seg.word.find("full speed") > -1:
                if self.speed == 0.2:
                    self.msg.linear.x = self.msg.linear.x*2
                    self.msg.angular.z = self.msg.angular.z*2
                    self.speed = 0.4
            if seg.word.find("half speed") > -1:
                if self.speed == 0.4:
                    self.msg.linear.x = self.msg.linear.x/2
                    self.msg.angular.z = self.msg.angular.z/2
                    self.speed = 0.2
            if seg.word.find("forward") > -1 or seg.word.find("ahead") > -1:
                self.soundhandle.say("Go Forward")
                self.msg.linear.x = self.speed
                self.msg.angular.z = 0
            if seg.word.find("left") > -1:
                self.soundhandle.say("turn left")
                if self.msg.linear.x != 0:
                    if self.msg.angular.z < self.speed:
                        self.msg.angular.z += 0.05
                else:
                    self.msg.angular.z = self.speed*2
            if seg.word.find("right") > -1:
                self.soundhandle.say("turn right")
                if self.msg.linear.x != 0:
                    if self.msg.angular.z > -self.speed:
                        self.msg.angular.z -= 0.05
                else:
                    self.msg.angular.z = -self.speed*2
            if seg.word.find("back") > -1:
                self.soundhandle.say("move back")
                self.msg.linear.x = -self.speed
                self.msg.angular.z = 0
            if seg.word.find("stop") > -1 or seg.word.find("halt") > -1:
                self.msg = Twist()
            #if seg.word.find("one") > -1:
                #self.soundhandle.say("confirm")
                #rospy.sleep(1)
                #if seg.word.find("yes"):
                 #   self.soundhandle.say("Go to position one")
                  #  goal.target_pose.header.frame_id = "map"
                   # goal.target_pose.header.stamp = rospy.Time.now()
                    #quat = quaternion_from_euler(0, 0, 0) 
                    #goal.target_pose.pose.position.x = -3.56
                    #goal.target_pose.pose.position.y = 1.67
                    #goal.target_pose.pose.position.z = 0
                    #goal.target_pose.pose.orientation.x = quat[0]
                    #goal.target_pose.pose.orientation.y = quat[1]
                    #goal.target_pose.pose.orientation.z = quat[2]
                    #goal.target_pose.pose.orientation.w = quat[3]
                    #self.client.send_goal(goal)
        self.pub_.publish(self.msg)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    # parser = argparse.ArgumentParser(
    #     description='Control ROS turtlebot using pocketsphinx.')
    # parser.add_argument('--model', type=str,
    #     default='/usr/local/lib/python2.7/dist-packages/pocketsphinx/model/en-us',
    #     help='''acoustic model path
    #     (default: /usr/local/lib/python2.7/dist-packages/pocketsphinx/model/eu-us)''')
    # parser.add_argument('--lexicon', type=str,
    #     default='voice_cmd.dic',
    #     help='''pronunciation dictionary
    #     (default: voice_cmd.dic)''')
    # parser.add_argument('--kwlist', type=str,
    #     default='voice_cmd.kwlist',
    #     help='''keyword list with thresholds
    #     (default: voice_cmd.kwlist)''')
    # parser.add_argument('--rospub', type=str,
    #     default='mobile_base/commands/velocity',
    #     help='''ROS publisher destination
    #     (default: mobile_base/commands/velocity)''')

    # args = parser.parse_args()
    #ASRControl(args.model, args.lexicon, args.kwlist, args.rospub)
    ASRControl()



