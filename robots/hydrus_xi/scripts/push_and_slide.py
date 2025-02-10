#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
from aerial_robot_msgs.msg import FlightNav

if __name__ == "__main__":

    rospy.init_node("push_and_slide")

    link_num = rospy.get_param("~link_num", 4)
    duration = rospy.get_param("~duration", 6)
    joint_control_topic_name = rospy.get_param("~joint_control_topic_name", "joints_ctrl")
    mode_pub = rospy.Publisher("/hydrus_xi/imp_mode", UInt8, queue_size=1)
    pos_pub = rospy.Publisher("/hydrus_xi/pos_cmds", Point, queue_size=1)
    nav_pub = rospy.Publisher("/hydrus_xi/uav/nav", FlightNav, queue_size=1)

    # time.sleep(1)

    # mode = UInt8()
    # mode.data = 1
    # pos = Point()
    # pos.x = 0.0
    # pos.y = 0.6
    # pos.z = 0.0
    nav_msg = FlightNav()
    # nav_msg.pos_xy_nav_mode = 4 # pos_vel mode
    # nav_msg.target_pos_x = 1.2
    # nav_msg.pos_z_nav_mode = 4 
    # nav_msg.target_pos_z = 0.5
    # nav_msg.yaw_nav_mode = 4 
    # nav_msg.target_yaw = 1.57

    # mode_pub.publish(mode)
    # pos_pub.publish(pos)
    # nav_pub.publish(nav_msg) # go to the origin point
    # time.sleep(16)

    # nav_msg.target_pos_x = 0.7
    # pos.y = 0.8

    # pos_pub.publish(pos)
    # nav_pub.publish(nav_msg) # get close to the wall
    # time.sleep(4)

    round = 0

    while not rospy.is_shutdown():
        round += 1
        if round % 2 == 1: 
            nav_msg.target_pos_y = 1.25   # to right
            nav_msg.target_vel_y = 0.08 
            print(nav_msg)
        else: 
            nav_msg.target_pos_y = -1.25  # to left
            nav_msg.target_vel_y = -0.08
            print(nav_msg)
        nav_pub.publish(nav_msg)
        time.sleep(duration)


