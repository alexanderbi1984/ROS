from nav_msgs.msg import OccupancyGrid
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point 

class GridMap():
    def __init__(self):
        