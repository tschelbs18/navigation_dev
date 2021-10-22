#!/usr/bin/env python

import rospy
import cv2
import apriltag
import numpy as np
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)


def tag_callback(msg):
    
    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    
    april_tag_distance = 0.5
    
    if msg.detections:
        
        pose_msg.pose = msg.detections.matrix[:9]
        pose_msg.pose[8] -= april_tag_distance        
        
    else:
        
        pose_msg.pose = []


    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
