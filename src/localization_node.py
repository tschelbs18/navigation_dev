#!/usr/bin/env python

import rospy
import cv2
import apriltag
import numpy as np
import time
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)


def tag_callback(msg):

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp

    camera_distance = 0.07

    if msg.ids:

        # Get first 12 elements of detections which are rotation matrix and translation vector
        # Note we are only looking at the first april tag detected - may want to consider more than 1
        matrix = list(msg.detections[0].matrix[:12])

        # Get position of robot from the reference frame of an april tag
        rotation_matrix = matrix[:3] + matrix[4:7] + matrix[8:11]
        rotation_matrix = np.reshape(np.array(rotation_matrix), (3, 3))
        rotation_matrix_transpose = np.transpose(rotation_matrix)

        translation_vector = -1 * \
            np.array([matrix[3]] + [matrix[7]] + [matrix[11]+camera_distance])

        camera_pos = np.matmul(rotation_matrix_transpose, translation_vector)

        # Format the matrix so that right is positive x, up is positive y (irrelevant), straight distance between robot and april tag is z
        frame_fix = np.reshape(
            np.array([-1, 0, 0, 0, -1, 0, 0, 0, -1]), (3, 3))
        camera_pos = np.matmul(frame_fix, camera_pos)

        # Retrive orientation where 0 is looking at april tag, positive is looking to the right of april tag
        orientation = -(matrix[2] * (np.pi/2))

        # Return position, orientation, and april tag id
        pose_msg.pose.matrix = list(camera_pos) + [orientation] + [msg.ids[0]]

    else:

        pose_msg.pose.matrix = []

    # time.sleep(0.3)
    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
