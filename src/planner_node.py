#!/usr/bin/env python

import rospy
import cv2
import apriltag
import argparse
import time
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose


ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
move = 1.0
stop = 0.0


def parse_args():

    parser = argparse.ArgumentParser(description='Inputs for JetBot')
    parser.add_argument("--left_forward_speed", default=0.98)
    parser.add_argument("--right_forward_speed", default=0.95)
    parser.add_argument("--left_turn_speed", default=0.88)
    parser.add_argument("--right_turn_speed", default=0.85)
    args = parser.parse_args()
    print(args)
    return args


def move_forward(distance, left_speed, right_speed, forward_rate=0.415):
    duration = int(round(distance/forward_rate * 10))

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def right_turn(turn, left_speed, right_speed, turn_rate=3.02):
    duration = int(round(turn/turn_rate * 10))

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, -left_speed, right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def left_turn(turn, left_speed, right_speed, turn_rate=1.87):

    duration = int(round(turn/turn_rate * 10))

    for _ in range(duration):

        msg = Float32MultiArray()
        msg.data = [move, left_speed, -right_speed]
        ctrl_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, left_speed, right_speed]
    ctrl_pub.publish(msg)
    time.sleep(1.0)


def pose_callback(msg):

    cmd_msg = Float32MultiArray()
    # ctrl_pub.publish(cmd_msg)

    t_matrix = msg.pose

    if not t_matrix:
        print("Finding April Tag!")
        right_turn(15, args.left_turn_speed, args.right_turn_speed)
    else:

        r13, r31, x_translation, z_translation = t_matrix.matrix[
            2], t_matrix.matrix[8], t_matrix.matrix[3], t_matrix.matrix[11]
        print('r13: ' + str(r13))
        print('r31: ' + str(r31))
        print('x_trans: ' + str(x_translation))
        print('z_trans: ' + str(z_translation))
        if z_translation > 0.05:

            if x_translation < -0.05:
                print("Turning right!")
                right_turn(0.5, args.left_turn_speed, args.right_turn_speed)
            elif x_translation > 0.05:
                print("Turning left!")
                left_turn(0.5, args.left_turn_speed, args.right_turn_speed)
            else:
                if z_translation >= 0.2:
                    print("Moving forward fast!")
                    move_forward(
                        z_translation/5, args.left_forward_speed, args.right_forward_speed)
                else:
                    print("Moving forward slow")
                    move_forward(
                        z_translation/2, args.left_forward_speed, args.right_forward_speed)

        else:
            # Are these both meant to be left turns?
            if r13 > 0.05:
                left_turn(0.5, args.left_turn_speed, args.right_turn_speed)
            elif r13 < 0.05:
                left_turn(0.5, args.left_turn_speed, args.right_turn_speed)
            else:
                print('waypoint 1 reached!')
                time.sleep(5.0)


if __name__ == "__main__":

    args = parse_args()
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
