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


# Code for ground truth of walls and obstacles and functions
def e_dist(p1, p2):
    return ((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)**0.5


def get_closest_pt(p1, point_set):
    min_dist = 10000000000000000
    for p2 in point_set:
        dist_check = e_dist(p1, p2)
        if dist_check < min_dist:
            min_dist = dist_check
            closest = p2
    return closest


def v_check(p, obstacles, boundaries):
    if abs(e_dist(p, get_closest_pt(p, obstacles)) - e_dist(p, get_closest_pt(p, boundaries))) <= 0.11:
        return True
    else:
        return False


def dedupe_vpts(v_pts):
    deduped = []
    for pt in v_pts:
        if 2.9 <= pt[0] <= 5.1 and (pt[1] == 1.7 or pt[1] == 6.3):
            pass
        else:
            deduped.append(pt)
    return deduped


l_wall = [[0.0, y / 10.0] for y in range(0, 81)]
r_wall = [[8.0, y / 10.0] for y in range(0, 81)]
t_wall = [[x / 10.0, 8.0] for x in range(0, 81)]
b_wall = [[x / 10.0, 0.0] for x in range(0, 81)]

l_o_wall = [[3.0, y / 10.0] for y in range(35, 46)]
r_o_wall = [[5.0, y / 10.0] for y in range(35, 46)]
t_o_wall = [[x / 10.0, 4.5] for x in range(30, 51)]
b_o_wall = [[x / 10.0, 3.5] for x in range(30, 51)]
obstacle_walls = l_o_wall + r_o_wall + t_o_wall + b_o_wall
walls = l_wall + r_wall + t_wall + b_wall

v_pts = []
for y in range(10, 71):
    for x in range(10, 21):
        if v_check([x / 10.0, y / 10.0], obstacle_walls, walls):
            v_pts.append([x / 10.0, y / 10.0])
    for x in range(60, 71):
        if v_check([x / 10.0, y / 10.0], obstacle_walls, walls):
            v_pts.append([x / 10.0, y / 10.0])

for x in range(20, 61):
    for y in range(50, 71):
        if v_check([x / 10.0, y / 10.0], obstacle_walls, walls):
            v_pts.append([x / 10.0, y / 10.0])
    for y in range(10, 31):
        if v_check([x / 10.0, y / 10.0], obstacle_walls, walls):
            v_pts.append([x / 10.0, y / 10.0])

v_pts = dedupe_vpts(v_pts)
v_pts = [p for i, p in enumerate(v_pts) if i % 5 == 0]

ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
move = 0.0
stop = 1.0
april_tag_map = {
    0: [6.5, 8.0, "down"],
    1: [4.0, 8.0, "down"],
    2: [1.5, 8.0, "down"],
    3: [0.0, 6.5, "right"],
    4: [0.0, 4.0, "right"],
    5: [0.0, 1.5, "right"],
    6: [1.5, 0.0, "up"],
    7: [4.0, 0.0, "up"],
    8: [6.5, 0.0, "up"],
    9: [8.0, 1.5, "left"],
    10: [8.0, 4.0, "left"],
    11: [8.0, 6.5, "left"]}
waypoints = [[4.0, 6.2], [1.5, 4.5]]


def parse_args():

    parser = argparse.ArgumentParser(description='Inputs for JetBot')
    parser.add_argument("--left_forward_speed", default=0.93)
    parser.add_argument("--right_forward_speed", default=0.90)
    parser.add_argument("--left_turn_speed", default=0.83)
    parser.add_argument("--right_turn_speed", default=0.80)
    args = parser.parse_args()
    print(args)
    return args


def move_forward(duration, left_speed, right_speed):

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def right_turn(duration, left_speed, right_speed):

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def left_turn(duration, left_speed, right_speed):

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def get_real_position(x, y, orientation, tag_x, tag_y, tag_facing):
    if tag_facing == "down":
        real_x = tag_x - x
        real_y = tag_y - y
        real_orientation = (3.1415 / 2) - orientation
    elif tag_facing == "right":
        real_x = tag_x + y
        real_y = tag_y - x
        real_orientation = 3.1415 - orientation
    elif tag_facing == "up":
        real_x = tag_x + x
        real_y = tag_y + y
        real_orientation = (3.1415 * 3 / 2) - orientation
    elif tag_facing == "left":
        real_x = tag_x - y
        real_y = tag_y + x
        real_orientation = -1*orientation
    # Need to cap orientation at [-2pi, 2pi]?
    return real_x, real_y, real_orientation


def pose_callback(msg):

    # Assign global variables that keep track of our progress
    t_matrix = msg.pose
    global waypoints

    # Turn right until you find an april tag (given layout of HW4, turning right should be better)
    if len(t_matrix.matrix) == 0:
        print("Finding an April Tag!")
        right_turn(2, args.left_turn_speed, args.right_turn_speed)
    else:
        # Once found, determine position relative to april tag & convert to feet
        x, z, y, orientation = t_matrix.matrix[0] * 3.28084, t_matrix.matrix[1] * \
            3.28084, t_matrix.matrix[2] * 3.28084, t_matrix.matrix[3]
        tag_id = t_matrix.matrix[4]
        print("Tag ID: " + str(tag_id))

        # Get ground truth of tag location
        tag_x = april_tag_map[tag_id][0]
        tag_y = april_tag_map[tag_id][1]
        tag_facing = april_tag_map[tag_id][2]

        # Using the tag, calculate our real x & y & orientation based on tag perception
        x, y, orientation = get_real_position(
            x, y, orientation, tag_x, tag_y, tag_facing)
        print('x: ' + str(x))
        print('y: ' + str(y))
        print('orientation: ' + str(orientation))

        # Find closest voronoi point that is closer to the destination than the robot
        dest_dist = e_dist([x, y], waypoints[0])
        test_pts = [p for p in v_pts if e_dist(p, waypoints[0]) < dest_dist]
        closest_v_pt = get_closest_pt([x, y], test_pts)
        print("Closest path point: " + str(closest_v_pt))

        # Given our v point destination, determine distance needed to move or turn
        move_x = closest_v_pt[0] - x
        move_y = closest_v_pt[1] - y
        print("Move x: " + str(move_x) + ", Move y: " + str(move_y))
        needed_turn = orientation - math.atan(move_y / move_x)
        print("Needed turn: " + str(needed_turn))

        # Determine actions needed to reach nearest voronoi point (if more than an inch away)
        if e_dist(closest_v_pt, [x, y]) > .083:
            if abs(needed_turn) < 0.1:
                # Move forward towards next point
                print("Moving forward slow")
                move_forward(
                    1, args.left_forward_speed, args.right_forward_speed)

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
            # Check if voronoi point was destination point
            if closest_v_pt == waypoints[0]:
                print("Waypoint reached, current position is x = {}, y = {} with an error of {}".format(
                    x, y, np.sqrt(x**2 + y**2)))
                waypoints.pop(0)
                time.sleep(5.0)
            # Handle a potential getting stuck where the closest v point is not a destination
            # And we still need to keep moving
            else:
                print("Moving forward slow")
                move_forward(1, args.left_forward_speed,
                             args.right_forward_speed)


if __name__ == "__main__":

    args = parse_args()
    rospy.init_node('planner_node')
    # We set the queue size to 1 to only look at the latest image detection
    rospy.Subscriber("/current_pose", Pose, pose_callback, queue_size=1)
    rospy.spin()
