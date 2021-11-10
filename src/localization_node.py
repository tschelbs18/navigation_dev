#!/usr/bin/env python

import rospy
import cv2
import apriltag
import numpy as np
import time
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)

dt = 1

# Initializes variables
s = np.array([0, 0, 0])
F = np.eye(s.shape[0])
G = np.array([[dt*np.cos(s[2]), 0], [dt*np.sin(s[2]), 0], [0, dt]])
H = np.array([[-np.cos(s[2]), -np.sin(s[2]), 0],
             [np.sin(s[2]), -np.cos(s[2]), 0]])
sigma = 0.5*np.eye(s.shape[0])
Q = 0.1*np.eye(s.shape[0])
R = 0.1*np.eye(2)

v_t = 1
w_t = 0.2

move = 1

time.sleep(5)
circle_trajectory = [[v_t, 0], [0, w_t]]*10
circle_index = 0


def mahalanobis_distance(s, f, R, H):
    diff = f-np.matmul(H, s)
    return np.sqrt(np.matmul(np.matmul(np.transpose(diff), np.linalg.inv(R)), (diff)))


def update_kalman(s, f, H, sigma, R):
    S = np.matmul(np.matmul(H, sigma), np.transpose(H)) + R
    K = np.matmul(np.matmul(sigma, np.transpose(H)), np.linalg.inv(S))
    s = s + np.matmul(K, f-np.matmul(H, s))
    sigma = np.matmul(np.eye(s.shape[0])-np.matmul(K, H), sigma)


def tag_callback(msg):

    global move
    global s
    global G
    global sigma
    global F
    global Q
    global H
    global circle_index

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    d = [v_t, w_t]

    if move == 1:
        if circle_index < len(circle_trajectory):
            pose_msg.pose.matrix = circle_trajectory[circle_index]
            pose_pub.publish(pose_msg)
        move = 0
        circle_index += 1
        time.sleep(1)

    elif msg.ids:

        move = 1
        # Update G with new theta
        G[0][0] = dt*np.cos(s[2])
        G[1][0] = dt*np.sin(s[2])
        # Predict
        s = np.matmul(F, s) + np.matmul(G, d)
        print("s" + str(s))
        sigma = np.matmul(np.matmul(F, sigma), np.transpose(F)) + Q

        for i in msg.detections:

            f = np.array([i.matrix[3], i.matrix[11]])
            num_features = (s.shape[0]-3)/2
            m_d = []
            for j in range(1, num_features+1):
                H_temp = H
                H_temp[0][2*j+1] = np.cos(s[2])
                H_temp[0][2*j+2] = np.sin(s[2])
                H_temp[1][2*j+1] = -np.sin(s[2])
                H_temp[1][2*j+2] = np.cos(s[2])
                m_d.append(mahalanobis_distance(s, f, R, H_temp))

            if m_d and np.min(m_d) < 3.5:
                corresponding_feature = np.argmin(m_d)+1
                H_new = H
                H_new[0][2*corresponding_feature+1] = np.cos(s[2])
                H_new[0][2*corresponding_feature+2] = np.sin(s[2])
                H_new[1][2*corresponding_feature+1] = -np.sin(s[2])
                H_new[1][2*corresponding_feature+2] = np.cos(s[2])
                # Update
                update_kalman(s, f, H_new, sigma, R)
            elif m_d and np.min(m_d) >= 3.5:
                # Add new landmark
                s = np.append(s, [s[0]+f[0]*np.cos(s[2]),
                                  s[1]+f[1]*np.sin(s[2])])
                F = np.eye(s.shape[0])
                G = np.append(G, [[0, 0], [0, 0]], axis=0)
                H = np.append(H, [[0, 0], [0, 0]], axis=1)
                Q = 0.1*np.eye(s.shape[0])

    # time.sleep(0.3)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
