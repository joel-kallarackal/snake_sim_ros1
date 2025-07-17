#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from typing import Dict
from rolling_helix import rolling_helix


def gait(beta, A, k, n, w, t, delta):
    return (beta + A*np.sin(k*n - w*t + delta)) * (-1) ** np.floor(n / 2)

def get_param(i,even,odd):
    return (1-i%2)*even + (i%2)*odd

A_even = 0
A_odd = 0
k=0
i=0
execute_pole_climb = False
if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/snake/joint_commands', JointState, queue_size=10)
        rospy.init_node('controller', anonymous=True)
        rate = rospy.Rate(10)
        t=0
        while not rospy.is_shutdown():
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
            
            t+=1
            A_even=0.3
            A_odd=0.3
            
            head_acc_x = 0
            head_acc_y = -9.6
            pole_direction = 1
            if abs(head_acc_x) > abs(head_acc_y):
                A_odd*=pole_direction
                if head_acc_x > 0:
                    A_odd*=-1
            else:
                A_even*=pole_direction
                if head_acc_y < 0:
                    A_even *= -1
            
            beta_even = 0
            beta_odd = 0
            delta_even = np.pi/2
            delta_odd = 0
            msg.position = [gait(get_param(i,beta_even,beta_odd),
                                get_param(i,A_even,A_odd),
                                0,
                                i,
                                0.5,
                                t,
                                get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

