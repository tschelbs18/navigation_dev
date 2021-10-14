#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 


ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)


def pose_callback(msg):
    # TODO: estimate control actions 
    cmd_msg = Float32MultiArray()


    ctrl_pub.publish(cmd_msg)


if __name__ == "__main__":
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
