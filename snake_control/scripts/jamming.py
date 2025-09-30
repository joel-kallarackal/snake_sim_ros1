#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import csv

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/snake/joint_commands', JointState, queue_size=10)
        rospy.init_node('controller', anonymous=True)
        rate = rospy.Rate(30)
        t=0
        
        trajectory = []
        with open("/home/kallrax/biorobotics_lab/ros1_ws/src/snake_control/data/interpolated_points.csv", "r") as f:
            reader = csv.reader(f)
            for row in reader:
                q = [float(i) for i in row]
                trajectory.append(q)
        
        while not rospy.is_shutdown() and t<len(trajectory):
            msg = JointState()
            msg.name = ["reu_joint_out2in_0",
                        "reu_joint_out2in_1",
                        "reu_joint_out2in_2",
                        "reu_joint_out2in_3",
                        "reu_joint_out2in_4",
                        "reu_joint_out2in_5",
                        "reu_joint_out2in_6",
                        "reu_joint_out2in_7",
                        "reu_joint_out2in_8",
                        "reu_joint_out2in_9",
                        "reu_joint_out2in_10",
                        "reu_joint_out2in_11",
                        "reu_joint_out2in_12",
                        "reu_joint_out2in_13",
                        "reu_joint_out2in_14",
                        "reu_joint_out2in_15",]
            
            msg.position = trajectory[t]
            pub.publish(msg)
            
            t+=1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

