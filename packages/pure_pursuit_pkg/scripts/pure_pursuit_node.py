#!/usr/bin/env python
import math
import rospy
# Import the different message structures that we will use.
from duckietown_msgs.msg import Segment, SegmentList, Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped
import time
import numpy as np

class pure_pursuit(object):
    # ##########################################################################
    # ############################## CONSTRUCTOR ############################# #
    # ##########################################################################
    def __init__(self):

        self.node_name = rospy.get_name()

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_radius_limit = rospy.Publisher("~radius_limit", BoolStamped, queue_size=1)
        self.pub_actuator_limits_received = rospy.Publisher("~actuator_limits_received", BoolStamped, queue_size=1)

        # Subscriptions
       # self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.PoseHandling, "lane_filter", queue_size=1) # Need to build callback. Might not be necessary
        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.updateWheelsCmdExecuted, queue_size=1) # Need to build callback
        self.sub_actuator_limits = rospy.Subscriber("~actuator_limits", Twist2DStamped, self.updateActuatorLimits, queue_size=1) # Need to build callback
        self.pub_seglist_filtered = rospy.Subscriber("~seglist_filtered",SegmentList, self.updateCommands, queue_size=1)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)


    # ##########################################################################
    # ######################### MAIN CONTROL FUNCTION ######################## #
    # ##########################################################################
    def updateCommands(self, segListMsg):
        v_bar = 0.3
        omega = 0
        L = 0.1 # Look-ahead distance
        L_tol = 0.05 # tolerance on segments which are in lookahead distance.

        segAvgx = 0
        segAvgy = 0
        segnum = 0
        for segment in segListMsg.segments:
            segx = (segment.points[0].x + segment.points[1].x)/2
            segy = (segment.points[0].y + segment.points[1].y)/2
            segdist = math.sqrt(segx ** 2 + segy ** 2)
            if (segment.color == segment.YELLOW): # Then this segment is near our look-ahead distance.
                segAvgx = segAvgx + segx
                segAvgy = segAvgy + segy
                segnum = segnum + 1
        
        if segnum != 0:
            segAvgx = segAvgx/segnum
            segAvgy = segAvgy/segnum
            L = math.sqrt(segAvgx ** 2 + segAvgy ** 2)
            sinAlpha = segAvgy
            omega = 1.5*2*v_bar*sinAlpha/L
        else:
            omega = 0

        # Publish the command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = v_bar
        car_control_msg.omega = omega
        self.pub_car_cmd.publish(car_control_msg)
        print("Angular Velocity: ", omega,"  L: ", L)








    # ########################## HOUSEKEEPING FUNCTIONS ##############################
    def updateWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd

    def updateActuatorLimits(self, msg_actuator_limits):
        self.actuator_limits = msg_actuator_limits
        rospy.logdebug("actuator limits updated to: ")
        rospy.logdebug("actuator_limits.v: " + str(self.actuator_limits.v))
        rospy.logdebug("actuator_limits.omega: " + str(self.actuator_limits.omega))
        msg_actuator_limits_received = BoolStamped()
        msg_actuator_limits_received.data = True
        self.pub_actuator_limits_received.publish(msg_actuator_limits_received)


    # ############################# CUSTOM SHUTDOWN ##################################
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_wheels_cmd_executed.unregister()
        self.sub_actuator_limits.unregister()


        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.pub_car_cmd.publish(car_control_msg)

        rospy.sleep(0.5)    #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


if __name__ == '__main__':
    rospy.init_node('pure_pursuit', anonymous=False)
    pure_pursuit_node = pure_pursuit()
    # if a safe shutdown is necessary
    #rospy.on_shutdown(lane_filter_node.onShutdown) 
    rospy.spin()  