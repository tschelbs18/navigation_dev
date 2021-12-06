#!/usr/bin/env python

import rospy
import cv2
import apriltag
import argparse
import time
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose


# Euclidean Distance
def e_dist(p1, p2):
    return ((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)**0.5


# Initialize publisher and waypoints
ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
move = 0.0
stop = 1.0
start = [2, 2]
robot_width = .4583  # feet
robot_length = .4167  # feet
c_size = [4, 4]

# Theoretically traverse the space to plan path
waypoints = []
position = start
finish = [x1 + x2 for x1, x2 in zip(start, c_size)]
while position != finish and position[1] <= finish[1]:
    x, y = position
    # Turn right
    if x == start[0]:
        waypoints.append([finish[0], y])
        waypoints.append([finish[0], y + 0.25])
    elif x == finish[0]:
        waypoints.append([start[0], y])
        waypoints.append([start[0], y + 0.25])
    position = waypoints[-1]
# Remove unnecessary turn at the end
waypoints.pop()


def parse_args():
    # Parse default inputs for speed
    parser = argparse.ArgumentParser(description='Inputs for JetBot')
    parser.add_argument("--left_forward_speed", default=0.93)
    parser.add_argument("--right_forward_speed", default=0.90)
    parser.add_argument("--left_turn_speed", default=0.83)
    parser.add_argument("--right_turn_speed", default=0.80)
    args = parser.parse_args()
    print(args)
    return args


def move_forward(duration, left_speed, right_speed):
    # Code for moving forward
    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def right_turn(duration, left_speed, right_speed):
    # Code for turning right
    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def left_turn(duration, left_speed, right_speed):
    # Code for turning left
    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def pose_callback(msg):

    # Assign global variables that keep track of our progress
    t_matrix = msg.pose
    global waypoints

    # If waypoints all found, do nothing (or... find untouched points and cover them)
    if len(waypoints) == 0:
        pass
    # Turn right until you find an april tag (given layout of HW4, turning right should be better)
    elif len(t_matrix.matrix) == 0:
        print("Finding an April Tag!")
        # right_turn(1, args.left_turn_speed, args.right_turn_speed)
    else:
        # Once found, determine position relative to april tag & convert to feet
        x, y, orientation = t_matrix.matrix[0], t_matrix.matrix[1], t_matrix.matrix[2]
        print('x: ' + str(x))
        print('y: ' + str(y))
        print('orientation: ' + str(orientation))

        # Given our next waypoint destination, determine distance needed to move or turn (EDIT ALL BELOW)
        destination = waypoints[0]
        move_x = destination[0] - x
        move_y = destination[1] - y
        print("Move x: " + str(move_x) + ", Move y: " + str(move_y))
        needed_turn = math.atan2(move_y, move_x) - orientation
        # Ensure we don't unncessarily turn in one direction
        if needed_turn >= math.pi:
            needed_turn = -2 * math.pi + needed_turn
        elif needed_turn <= -1 * math.pi:
            needed_turn = 2 * math.pi + needed_turn
        print("Needed turn: " + str(needed_turn))

        # Determine actions needed to reach next waypoint (if more than .1 feet away)
        print("Distance Remaining" + str(e_dist(destination, [x, y])))
        if e_dist(destination, [x, y]) > .2:
            # Big move for more than 0.5 feet away
            if abs(needed_turn) < 0.20 and e_dist(destination, [x, y]) > .5:
                # Move forward towards next point
                print("Moving forward")
                move_forward(
                    3, args.left_forward_speed, args.right_forward_speed)
            # Smaller move for less than 6 inches away
            elif abs(needed_turn) < 0.3:
                # Move forward towards next point
                print("Moving forward")
                move_forward(
                    2, args.left_forward_speed, args.right_forward_speed)

            elif needed_turn > 0:
                # Align to next point by turning left
                print("Turning left!")
                left_turn(1, args.left_turn_speed, args.right_turn_speed)

            elif needed_turn < 0:
                # Align to next point by turning right
                print("Turning right!")
                right_turn(1, args.left_turn_speed, args.right_turn_speed)
            else:
                print("Unaccounted situation! Help! x = {}, y = {}, orientation = {}".format(
                    x, y, orientation))
        else:
            # Arrived at destination
            print("Waypoint reached, current position is x = {}, y = {} with an error of {}".format(
                x, y, np.sqrt(x**2 + y**2)))
            waypoints.pop(0)


if __name__ == "__main__":

    args = parse_args()
    rospy.init_node('planner_node')
    # We set the queue size to 1 to only look at the latest image detection
    rospy.Subscriber("/current_pose", Pose, pose_callback, queue_size=1)
    rospy.spin()
