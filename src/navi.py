#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math
import numpy
rospy.set_param('range_max', 100)

pub_ = None
regions_ = {
    'right': 0,
    'front_right': 0,
    'front': 0,
    'front_left': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def clbk_laser(msg):
	#sensor ranges arrary is 640 in the 180-degree range.
    global regions_
    ranges = numpy.array(msg.ranges)
    # for item in range(640):
    #     if math.isnan(ranges[item]):
    #         ranges[item] = 9
            


    regions_ = {
        'right':  min(numpy.nanmin(ranges[0:127]), 10),
        'front_right': min(numpy.nanmin(ranges[128:255]), 10),
        'front':  min(numpy.nanmin(ranges[256:383]), 10),
        'front_left':  min(numpy.nanmin(ranges[384:511]), 10),
        'left':   min(numpy.nanmin(ranges[512:639]), 10),}
    print(regions_)
    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        # print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
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
    #     regions_['left'] = regions_['front_left']
    # if math.isnan(regions_['right']):
    #     regions_['right'] = regions_['front_right']
    # if math.isnan(regions_['front_left']):
    #     regions_['front_left'] = max(regions_['left'],regions_['front'])
    # if math.isnan(regions_['front_right']):
    #     regions_['front_right'] = max(regions_['right'],regions_['front'])
    # if math.isnan(regions_['front']):
    #     regions_['front'] = max(regions_['front_left'],regions_['front_right'])
    
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['front_left'] > d and regions['front_right'] < d and regions['right'] < d and regions['front_right'] > 0.8 and regions['right'] > 0.8:
        state_description = 'case 3 - front_right'
        change_state(2)
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 4 - front_left'
        change_state(0)
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 5 - front and front_right'
        change_state(1)
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 6 - front and front_left'
        change_state(1)
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 7 - front and front_left and front_right'
        change_state(1)
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 8 - front_left and front_right'
        change_state(0)
    elif regions['front'] > 2 or regions['front_left'] > 2 and regions['front_right'] < 0.8 and regions['right'] < 0.8:
        state_description = 'case 9 - door on right'
        change_state(1)
    elif math.isnan(regions_['right']) and math.isnan(regions_['front_right']):
        state_description = 'case 10 - hit wall'
        change_state(1)
    else:
        state_description = 'unknown case'
        change_state(1)
        rospy.loginfo(regions)

def find_wall():
    print("find wall")
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = -0.1
    return msg

def turn_left():
    print("turn left")
    msg = Twist()
    #msg.linear.x = 0+numpy.random.randint(-1,2)*0.5
    #msg.angular.z = 0+numpy.random.randint(0,2)*0.5 
    msg.angular.z = 0.5  
    return msg

def follow_the_wall():
    global regions_
    print("follow the wall")
    msg = Twist()
    msg.linear.x = 0.3
    return msg

def do_nothing():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
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
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()