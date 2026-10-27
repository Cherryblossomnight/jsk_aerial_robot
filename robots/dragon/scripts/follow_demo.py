#!/usr/bin/env python

import numpy as np
import sys
import time
import rospy
import math
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf

def eff_cb(data):
    global pointStamped
    pointStamped = data


def pose_cb(data):
    global pose
    pose = data.pose.pose


if __name__=="__main__":
    rospy.init_node("dragon_follow_demo")
    pointStamped = PointStamped()
    pointStamped.point.x = 1.0
    pose = Pose()


    reached = False

    rospy.Subscriber("dragon/effortor_from_cog", PointStamped, eff_cb)
    
    rospy.Subscriber("dragon/uav/cog/odom", Odometry, pose_cb)
 
    pose_pub = rospy.Publisher("/dragon/target_pose", PoseStamped, queue_size=1)
    


    while not rospy.is_shutdown():

        rotation_matrix = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        point = np.array([pointStamped.point.x, pointStamped.point.y, pointStamped.point.z])
        point_new = np.dot(rotation_matrix[:3, :3], point)
        print(rotation_matrix[:3, :3])
        print(point_new)

        pose1 = PoseStamped()
        pose1.pose.orientation = pose.orientation
        pose1.pose.position.x = 5.0 - point_new[0]
        pose1.pose.position.y = 0.0 - point_new[1]
        pose1.pose.position.z = 3.0 - point_new[2]

        pose2 = PoseStamped()
        pose2.pose.orientation = pose.orientation
        pose2.pose.position.x = 5.0 - point_new[0]
        pose2.pose.position.y = 5.0 - point_new[1]
        pose2.pose.position.z = 3.0 - point_new[2]

        if abs(5.0 - point_new[0]- pose.position.x)<0.01 and abs(-pose.position.y -  point_new[1])<0.01 and abs(3.0 - point_new[2] - pose.position.z)<0.01:
            reached = True
        if reached:
            pose_pub.publish(pose2)
            print("reached")
            print(pose2.pose.position)

        else:
            pose_pub.publish(pose1)
            print(pose1.pose.position)

        

        time.sleep(1)



