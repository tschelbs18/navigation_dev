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

    april_tag_distance = 0.5
    camera_distance = 0.07

    if msg.ids:

        matrix = list(msg.detections[0].matrix[:12])
        # matrix = [-0.9987146795786737, -0.026355616586157645, -0.043293998064751524, -0.12531718125760102, 0.029071604537249992, -0.9975677793426647, -0.06335114384898045, 0.05556224903340525, -0.04151903905074156, -0.0645283433206519, 0.9970517851669466, 0.49853897165782207]
        # matrix = [-0.9514170569561136, -0.026837693097907206, 0.30673330755258854, -0.03189151908336987, 0.0059397328149474685, -0.9976084949976138, -0.06886225586403497, 0.04629209875574662, 0.3078478574020864, -0.06369481091721149, 0.9493011470314177, 0.5615473011789451]
        rotation_matrix = matrix[:3] + matrix[4:7] + matrix[8:11]
        rotation_matrix = np.reshape(np.array(rotation_matrix), (3, 3))
        rotation_matrix_transpose = np.transpose(rotation_matrix)

        translation_vector = -1 * \
            np.array([matrix[3]] + [matrix[7]] + [matrix[11]+camera_distance])

        camera_pos = np.matmul(rotation_matrix_transpose, translation_vector)

        frame_fix = np.reshape(
            np.array([-1, 0, 0, 0, -1, 0, 0, 0, -1]), (3, 3))
        camera_pos = np.matmul(frame_fix, camera_pos)

        orientation = -(matrix[2] * (np.pi/2))

        pose_msg.pose.matrix = list(camera_pos) + [orientation] + [msg.ids[0]]

    else:

        pose_msg.pose.matrix = []

    time.sleep(0.3)
    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
