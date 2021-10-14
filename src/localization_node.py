#!/usr/bin/env python

import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)


def tag_callback(msg):
    # TODO: implement localization logic 

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp

    # TODO
    # pose_msg.pose = [R11, R12, R13, t1,
    #                  R21, R22, R23, t2,
    #                  R31, R32, R33, t3]

    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
