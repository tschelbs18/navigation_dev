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
waypoint = 0
waypoint_reached = 0
april_tag_distance = 0.5
lap_complete = 0


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

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def right_turn(duration, turn, left_speed, right_speed, turn_rate=2.52):

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def left_turn(duration, turn, left_speed, right_speed, turn_rate=1.87):

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def pose_callback(msg):

    # Assign global varibales that keep track of our progress
    t_matrix = msg.pose
    global waypoint
    global waypoint_reached
    global lap_complete

    # Turn left until you find an april tag
    if len(t_matrix.matrix) == 0 or t_matrix.matrix[4] != waypoint:
        print("Finding April Tag!")
        left_turn(3, 0.2, args.left_turn_speed, args.right_turn_speed)
    else:
        # Once found, determine position relative to april tag
        x, y, z, orientation = t_matrix.matrix[0], t_matrix.matrix[1], t_matrix.matrix[2], t_matrix.matrix[3]
        if lap_complete == 1:
            z = april_tag_distance
        print('x: ' + str(x))
        print('z: ' + str(z))
        print('orientation: ' + str(orientation))
        waypoint_reached = 0
        # Determine actions needed to reach waypoint
        if z > april_tag_distance:
            if x > 0 and orientation < 0 or x < 0 and orientation > 0:
                # Move forward towards april tag
                if z > april_tag_distance + 0.2:
                    print("Moving forward fast!")
                    move_forward(
                        3, (z-april_tag_distance)/5, args.left_forward_speed, args.right_forward_speed)
                else:
                    print("Moving forward slow")
                    move_forward(
                        1, (z-april_tag_distance)/2, args.left_forward_speed, args.right_forward_speed)

            elif x < 0 and orientation < 0:
                # Align to april tag by turning left
                print("Turning left!")
                left_turn(1, 0.2, args.left_turn_speed, args.right_turn_speed)

            elif x > 0 and orientation > 0:
            
                # Align to april tag by turning right
                print("Turning right!")
                right_turn(1, 0.2, args.left_turn_speed, args.right_turn_speed)

            else:
                print("Unaccounted situation! Help! x = {}, z = {}, orientation = {}".format(
                    x, z, orientation))

        else:
            
            # Turn to align with april tag
            if orientation > 0.1:
                print("Turning right!")
                right_turn(1, 0.1, args.left_turn_speed, args.right_turn_speed)

            # Turn to align with april tag
            elif orientation < -0.1:
                print("Turning left!")
                left_turn(1, 0.1, args.left_turn_speed, args.right_turn_speed)

            else:
                '''if waypoint_reached == 0:'''
                # Reached april tag.  Determine next waypoint
                print("Waypoint reached, current position is x = {}, z = {} with an error of {}".format(
                    x, z, np.sqrt(x**2 + z**2)))
                time.sleep(5.0)
                waypoint += 1
                if waypoint == 4:
                    waypoint = 0
                    lap_complete = 1
                waypoint_reached = 1
                '''
                else:
                    print("Turning left!")
                    left_turn(1, 0.3, args.left_turn_speed, args.right_turn_speed)
                '''


if __name__ == "__main__":

    args = parse_args()
    rospy.init_node('planner_node')
    # Could we change this to re-initialize our subscriber after doing some action? Clear out the queue?
    rospy.Subscriber("/current_pose", Pose, pose_callback, queue_size=1)
    rospy.spin()
