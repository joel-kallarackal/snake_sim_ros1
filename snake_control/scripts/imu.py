#!/usr/bin/env python

import pybullet as p
import numpy as np
import rospy
from sensor_msgs.msg import Imu
import tf

global prev_time, prev_velocity, prev_orientation



def imu_publisher():
    rospy.init_node('imu_simulator')
    
    global prev_time, prev_velocity, prev_orientation
    prev_time = rospy.Time.now().to_sec()
    prev_velocity = [0, 0, 0]
    prev_orientation = [0, 0, 0, 1]  # quaternion

    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    cid = p.connect(p.GUI)
    if cid < 0:
        print("Could not connect to shared memory PyBullet server")
        exit()

    robot_id = 1  # your loaded robot in PyBullet
    link_index = -1  # base link usually

    while not rospy.is_shutdown():
        imu_data = compute_imu(robot_id, link_index)
        if imu_data is None:
            continue

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = imu_data['linear_acceleration'][0]
        imu_msg.linear_acceleration.y = imu_data['linear_acceleration'][1]
        imu_msg.linear_acceleration.z = imu_data['linear_acceleration'][2]

        imu_msg.angular_velocity.x = imu_data['angular_velocity'][0]
        imu_msg.angular_velocity.y = imu_data['angular_velocity'][1]
        imu_msg.angular_velocity.z = imu_data['angular_velocity'][2]

        imu_msg.orientation.x = imu_data['orientation'][0]
        imu_msg.orientation.y = imu_data['orientation'][1]
        imu_msg.orientation.z = imu_data['orientation'][2]
        imu_msg.orientation.w = imu_data['orientation'][3]

        imu_pub.publish(imu_msg)
        rate.sleep()


imu_publisher()