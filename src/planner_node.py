#!/usr/bin/env python

import rospy
import cv2
import apriltag
import argparse
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose


ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
move = 0.0
stop = 1.0


def parse_args():

    parser = argparse.ArgumentParser(description='Inputs for JetBot')
    parser.add_argument("--left_forward_speed", default=0.93)
    parser.add_argument("--right_forward_speed", default=0.90)
    parser.add_argument("--left_turn_speed", default=0.83)
    parser.add_argument("--right_turn_speed", default=0.8)
    args = parser.parse_args()
    print(args)
    return args


def move_forward(duration, distance, left_speed, right_speed, forward_rate=0.415):
    # Note for Hmwk2, we ignore distance and rate - move in small bursts of time instead

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def right_turn(duration, turn, left_speed, right_speed, turn_rate=2.52):
    # Note for Hmwk2, we ignore distance and rate - move in small bursts of time instead

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def left_turn(duration, turn, left_speed, right_speed, turn_rate=1.87):
    # Note for Hmwk2, we ignore distance and rate - move in small bursts of time instead

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def pose_callback(msg):
    # Make movements
    t_matrix = msg.pose.matrix
    print(t_matrix)
    if t_matrix[0] != 0:
        print("Moving forward")
        move_forward(3, 0, 0.87, .85)
    else:
        print("Turning left")
        left_turn(1, 0, 0.87, 0.85)


if __name__ == "__main__":

    args = parse_args()
    rospy.init_node('planner_node')
    # We set the queue size to 1 to only look at the latest image detection
    rospy.Subscriber("/current_pose", Pose, pose_callback, queue_size=1)
    rospy.spin()
