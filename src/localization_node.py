#!/usr/bin/env python

import rospy
import cv2
import apriltag
import numpy as np
import time
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)

april_tag_map = [[1.5, 0.0, np.pi/2, 2],
                 [4.0, 0.0, np.pi/2, 1],
                 [6.5, 0.0, np.pi/2, 2],
                 [8.0, 1.5, np.pi, 1],
                 [8.0, 4.0, np.pi, 2],
                 [8.0, 6.5, np.pi, 1],
                 [6.5, 8.0, 3*np.pi/2, 1],
                 [4.0, 8.0, 3*np.pi/2, 2],
                 [1.5, 8.0, 3*np.pi/2, 1],
                 [0.0, 6.5, 0.0, 1],
                 [0.0, 4.0, 0.0, 2],
                 [0.0, 1.5, 0.0, 1]]

robot_pos = [7.0, 7.0, np.pi]


def tag_callback(msg):

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    global robot_pos

    camera_distance = 0.07

    if msg.ids:

        # Get first 12 elements of detections which are rotation matrix and translation vector
        # Note we only consider the april tag found with the lowest relative orientation, to avoid confusion
        x_min = [abs(i.matrix[3]) for i in msg.detections]
        x_argmin = np.argmin(x_min)
        matrix = list(msg.detections[x_argmin].matrix[:12])

        # Get position of april tag from robot
        april_tag = [robot_pos[0] + 3.28084*matrix[11]*np.cos(robot_pos[2]) + 3.28084*matrix[3]*np.sin(robot_pos[2]),
                     robot_pos[1] + 3.28084*matrix[11]*np.sin(robot_pos[2]) + 3.28084*matrix[3]*np.cos(robot_pos[2])]

        april_tag_min_distance = 1000
        april_tag_min_index = -1
        for j, v in enumerate(april_tag_map):
            april_tag_distance = np.sqrt(
                (v[0]-april_tag[0])**2 + (v[1]-april_tag[1])**2)
            if april_tag_distance < april_tag_min_distance:
                april_tag_min_distance = april_tag_distance
                april_tag_min_index = j

        print(april_tag_map[april_tag_min_index])
        print(april_tag)

        if april_tag_min_distance < 0.7 and april_tag_map[j][3] == msg.ids[x_argmin]:

            rotation_matrix = matrix[:3] + matrix[4:7] + matrix[8:11]
            rotation_matrix = np.reshape(np.array(rotation_matrix), (3, 3))
            rotation_matrix_transpose = np.transpose(rotation_matrix)

            translation_vector = -1 * \
                np.array([matrix[3]] + [matrix[7]] +
                         [matrix[11]+camera_distance])

            camera_pos = np.matmul(
                rotation_matrix_transpose, translation_vector)

            # Format the matrix so that right is positive x, up is positive y (irrelevant), straight distance between robot and april tag is z
            frame_fix = np.reshape(
                np.array([-1, 0, 0, 0, -1, 0, 0, 0, -1]), (3, 3))
            camera_pos = np.matmul(frame_fix, camera_pos)

            robot_pos[0] = april_tag_map[april_tag_min_index][0] + 3.28084*camera_pos[2]*np.cos(
                april_tag_map[april_tag_min_index][2]) + 3.28084*camera_pos[0]*np.sin(april_tag_map[april_tag_min_index][2])
            robot_pos[1] = april_tag_map[april_tag_min_index][1] + 3.28084*camera_pos[2]*np.sin(
                april_tag_map[april_tag_min_index][2]) - 3.28084*camera_pos[0]*np.cos(april_tag_map[april_tag_min_index][2])

            # Retrieve orientation where 0 is looking at april tag, positive is looking to the right of april tag
            robot_pos[2] = -1*np.pi + april_tag_map[april_tag_min_index][2] + \
                matrix[2] * (np.pi/-2)
            # Keep robot_pos bounded by [-2pi, 2pi]
            if robot_pos[2] > 2*np.pi:
                robot_pos[2] = robot_pos[2] - 2*np.pi
            elif robot_pos[2] < -2*np.pi:
                robot_pos[2] = robot_pos[2] + 2*np.pi

            # Return position, orientation, and april tag id
            pose_msg.pose.matrix = robot_pos
            print("Robot Position {}".format(robot_pos))

        else:
            print("Robot Position {}".format(robot_pos))
            print("I might be lost")

    else:

        pose_msg.pose.matrix = []

    # time.sleep(0.3)
    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
