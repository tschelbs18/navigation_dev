#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


# initialization
if __name__ == '__main__':

    # setup ros node
    rospy.init_node('jetbot_test')
    ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)

    move = 0.0
    stop = 1.0

    # forward
    shape = "circle"

    # Moving in a circle
    if shape == "circle":
        speed_l = -0.68
        speed_r = -0.80
        wait_time = 1
        moving_time = 3
        steps = 12
        for j in range(steps):
            for i in range(moving_time):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(wait_time)

    elif shape == "figure-eight":
        # Need to change the speeds for a tighter circle
        # should do 3/4 left circle
        # full right circle
        # 1/4 left circle
        # start position will be different?
        speed_l = -0.68
        speed_r = -0.80
        wait_time = 1
        moving_time = 3
        steps = 6
        for j in range(steps):
            for i in range(moving_time):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(wait_time)

        speed_l = -0.83
        speed_r = -0.65
        steps = 12
        for j in range(steps):
            for i in range(moving_time):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(wait_time)

        speed_l = -0.68
        speed_r = -0.80
        steps = 3
        for j in range(steps):
            for i in range(moving_time):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(wait_time)
