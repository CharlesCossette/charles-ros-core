#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
import time
import numpy as np

class purePursuit(object):
    # ##########################################################################
    # ############################## CONSTRUCTOR ############################# #
    # ##########################################################################
    def __init__(self)
        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_radius_limit = rospy.Publisher("~radius_limit", BoolStamped, queue_size=1)


if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous=False)
    pure_pursuit_node = purePursuit()
    # if a safe shutdown is necessary
    #rospy.on_shutdown(lane_filter_node.onShutdown) 
    rospy.spin()  