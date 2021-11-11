#!/usr/bin/env python

import rospy
import cv2
import apriltag
import numpy as np
import time
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)

dt = .3

# Initializes variables
path = "circle"
# Set starting position based on path (circle or figure 8)
if path == "circle":
    s = np.array([6, 4, 3.1415/2])
else:
    s = np.array([5, 5, 3.1415/2])
F = np.eye(s.shape[0])
G = np.array([[dt*np.cos(s[2]), 0], [dt*np.sin(s[2]), 0], [0, dt]])
H = np.array([[-np.cos(s[2]), -np.sin(s[2]), 0],
             [np.sin(s[2]), -np.cos(s[2]), 0]])
sigma = 0.5*np.eye(s.shape[0])
Q = 0.1*np.eye(s.shape[0])
R = 0.1*np.eye(2)

# These are our calibrated distance and rotational velocities in ft / sec and radians / sec
if path == "circle":
    v_t = 1.5 / 12 / .3
    w_t = 0.018 / .1
else:
    v_t = 1.5 / 12 / .3
    w_t = 0.018 / .1

move = 1

time.sleep(5)
# This is how many movements it take to functionally create a circle, our strategy is to move in a 40 sided polygon, creating a circle
if path == "circle":
    trajectory = [[v_t, 0], [0, w_t]]*80
else:
    trajectory = [[v_t, 0], [0, w_t]] * 30 + \
        [[v_t, 0], [0, -w_t]] * 40 + [[v_t, 0], [0, w_t]] * 10
idx = 0


def mahalanobis_distance(s, f, R, H):
    # Distance calculation
    diff = f-np.matmul(H, s)
    # print("f= " + str(f))
    # print("H= " + str(H))
    # print("s= " + str(s))
    # print("H*s= " + str((np.matmul(H, s))))
    # print("MD Diff: " + str(diff))
    return np.sqrt(np.matmul(np.matmul(np.transpose(diff), np.linalg.inv(R)), (diff)))


def update_kalman(s, f, H, sigma, R):
    # Update Kalman filter
    S = np.matmul(np.matmul(H, sigma), np.transpose(H)) + R
    K = np.matmul(np.matmul(sigma, np.transpose(H)), np.linalg.inv(S))
    s = s + np.matmul(K, f-np.matmul(H, s))
    sigma = np.matmul(np.eye(s.shape[0])-np.matmul(K, H), sigma)
    return s, sigma


def tag_callback(msg):

    global move
    global s
    global G
    global sigma
    global F
    global Q
    global H
    global idx

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    d = trajectory[idx]

    if move == 1:
        if idx < len(trajectory):
            pose_msg.pose.matrix = trajectory[idx]
            pose_pub.publish(pose_msg)
        elif idx == len(trajectory):
            fi = open("s_matrix.txt", "w")
            fi.write(str(s))
            fi.close()
        move = 0
        idx += 1
        time.sleep(1)

    elif msg.ids:

        move = 1
        # Update G with new theta
        G[0][0] = dt*np.cos(s[2])
        G[1][0] = dt*np.sin(s[2])
        H[0][0] = -np.cos(s[2])
        H[0][1] = -np.sin(s[2])
        H[1][0] = np.sin(s[2])
        H[1][1] = -np.cos(s[2])
        # Predict
        s = np.matmul(F, s) + np.matmul(G, d)
        print("s:" + str(s))
        sigma = np.matmul(np.matmul(F, sigma), np.transpose(F)) + Q

        for i in msg.detections:

            md_threshold = 6
            f = np.array([i.matrix[11]*3.28, i.matrix[3]*3.28])
            num_features = (s.shape[0]-3)/2
            m_d = []
            for j in range(1, num_features+1):
                H_temp = np.copy(H)
                H_temp[0][2*j+1] = np.cos(s[2])
                H_temp[0][2*j+2] = np.sin(s[2])
                H_temp[1][2*j+1] = -np.sin(s[2])
                H_temp[1][2*j+2] = np.cos(s[2])
                m_d.append(mahalanobis_distance(s, f, R, H_temp))

            if m_d:
                print("Min M_D: " + str(np.min(m_d)))
            if m_d and np.min(m_d) < md_threshold:
                corresponding_feature = np.argmin(m_d)+1
                print("OLD LANDMARK FOUND: " + str(corresponding_feature))
                H_new = np.copy(H)
                H_new[0][2*corresponding_feature+1] = np.cos(s[2])
                H_new[0][2*corresponding_feature+2] = np.sin(s[2])
                H_new[1][2*corresponding_feature+1] = -np.sin(s[2])
                H_new[1][2*corresponding_feature+2] = np.cos(s[2])
                # Update
                s, sigma = update_kalman(s, f, H_new, sigma, R)
            elif m_d and np.min(m_d) >= md_threshold or not m_d:
                # Add new landmark
                print("NEW LANDMARK FOUND")
                s = np.append(s, [s[0] + f[0] * np.cos(s[2])-f[1]*np.sin(s[2]),
                                  s[1] + f[0]*np.sin(s[2])-f[1]*np.cos(s[2])])
                F = np.eye(s.shape[0])
                G = np.append(G, [[0, 0], [0, 0]], axis=0)
                H = np.append(H, [[0, 0], [0, 0]], axis=1)
                Q = 0.1*np.eye(s.shape[0])
                sigma = np.append(sigma, np.zeros((2, sigma.shape[0])), axis=0)
                sigma = np.append(sigma, np.zeros((sigma.shape[0], 2)), axis=1)
                sigma[-1][-1] = 0.5
                sigma[-2][-2] = 0.5
    time.sleep(1)
    # time.sleep(0.3)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback, queue_size=1)
    rospy.spin()
