#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math
import numpy

pub_ = None
regions_ = {
    'right': 0,
    'front': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'turn right',
    4: 'backup'
}

def clbk_laser(msg):
	#sensor ranges arrary is 640 in the 180-degree range.
    global regions_
    ranges = numpy.array(msg.ranges)
    
            


    regions_ = {
        'right':  min(numpy.nanmean(ranges[0:99]), 5),
        'front':  min(numpy.nanmean(ranges[270:369]), 5),
        'left':   min(numpy.nanmean(ranges[540:639]), 5),}
    print(regions_)
    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1
    # if math.isnan(regions_['left']):
    #     regions_['left'] = regions_['left']
    # if math.isnan(regions_['right']):
    #     regions_['right'] = regions_['right']
    # if math.isnan(regions_['left']):
    #     regions_['left'] = max(regions_['left'],regions_['front'])
    # if math.isnan(regions_['right']):
    #     regions_['right'] = max(regions_['right'],regions_['front'])
    # if math.isnan(regions_['front']):
    #     regions_['front'] = max(regions_['left'],regions_['right'])
    
    if regions['front'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 1 - nothing'
        if 2*d < regions['front'] <4 and 2*d < regions['left'] < 4 and regions['right'] >= 4:
            change_state(3)
        else:
            change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 2 - front'
        if regions['right'] < 2 * d:
            change_state(1)
        else:
            change_state(3)
    elif (regions['front'] > d ) and (regions['left'] > d ) and regions['right'] < d:
        state_description = 'case 3 - right'
        # change_state(2)
        if regions['right'] < float(d)/2:
            change_state(1)
        else:
            change_state(2)
    elif regions['front'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 4 - left'
        if regions['left'] < float(d)/2:
            change_state(4)
        else:
            change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 5 - front and right'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 6 - front and left'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 7 - front and left and right'
        change_state(1)
    elif regions['front'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 8 - left and right'
        change_state(2)
    # elif math.isnan(regions['front']):
    #     change_state(4)
    # elif regions['front'] > 2 or regions['left'] > 2 and regions['right'] < 0.8 and regions['right'] < 0.8:
    #     state_description = 'case 9 - door on right'
    #     change_state(1)
    # elif math.isnan(regions_['right']) and math.isnan(regions_['right']):
    #     state_description = 'case 10 - hit wall'
    #     change_state(1)
    else:
        state_description = 'unknown case'
        # change_state(1)
        rospy.loginfo(regions)

def find_wall():
	print("find wall")
	msg = Twist()
	msg.linear.x = numpy.random.uniform(0,0.3)
	msg.angular.z = numpy.random.uniform(-0.1,0)
	return msg

def turn_left():
	print("turn left")
	msg = Twist()
	msg.angular.z = 0.2
	return msg

def follow_the_wall():
    global regions_
    print("follow the wall")
    msg = Twist()
    msg.linear.x = 0.3
    return msg

def turn_right():
    print('through the door')
    msg = Twist()
    msg.linear.x = 0.4
    msg.angular.z = -0.9
    return msg

def backup():
    msg= Twist()
    msg.linear.x = -0.1
    return msg
def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 3:
            msg = turn_right()
        elif state_ == 4:
            msg = backup()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()