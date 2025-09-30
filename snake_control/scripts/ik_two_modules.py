#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import sys
import termios
import tty
import copy

def ik_2modules(r, p, y):
    theta1, theta2 = 0,0
    if p != np.pi/2 and p != -np.pi/2:
        theta1 = -p
        theta2 = y
    else:
        theta1 = -p
        if theta1 == np.pi/2:
            theta2 = 90 - r
        else:
            theta2 = -(90 - r)
        
    return theta1, theta2

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch

initial_pose = [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
yaw, pitch = 0, 0

head_acc = [0, -9]
yaw_sign, pitch_sign = 1,1
swap_yp = False
if abs(head_acc[0])>abs(head_acc[1]):
    if head_acc[0]>0:
        yaw_sign*=-1
        pitch_sign*=-1
else:
    # swap yaw and pitch in joystick
    swap_yp = True
    if head_acc[1]>0:
        yaw_sign*=-1
    else:
        pitch_sign*=-1
        
    


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/snake/joint_commands', JointState, queue_size=10)
        rospy.init_node('head_ik', anonymous=True)
        
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
        # msg.position = [1.1274298724979068, 0.5214787464862762, -1.0233078625570493, -0.7257993868875412, 0.878389797370735, 0.9011846961260116, -0.6984531028825454, -1.040642615382698, 0.490671287278658, 1.1386133975017492, -0.26332795585375135, -1.1911912565647602, 0.025486569808375298, 1.1962800792499158, 0.21337088534773876, -1.1536769902554687]
        # initial_pose = [1.1274298724979068, 0.5214787464862762, -1.0233078625570493, -0.7257993868875412, 0.878389797370735, 0.9011846961260116, -0.6984531028825454, -1.040642615382698, 0.490671287278658, 1.1386133975017492, -0.26332795585375135, -1.1911912565647602, 0.025486569808375298, 1.1962800792499158, 0.21337088534773876, -1.1536769902554687]
        
        msg.position = [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        prev_pose = [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        pub.publish(msg)
        
        rate = rospy.Rate(10)
        rate.sleep()
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
                        "reu_joint_out2in_15"]
            t+=1
        
            # IK Keyboard control 
            pitch_before = copy.copy(pitch)
            yaw_before = copy.copy(yaw)
            key = get_key()
            if swap_yp:
                if key=="q":
                    pitch+=0.05*pitch_sign
                elif key=="a":
                    pitch-=0.05*pitch_sign
                elif key=="w":
                    yaw+=0.05*yaw_sign
                elif key=="s":
                    yaw-=0.05*yaw_sign
            else:
                if key=="q":
                    yaw+=0.05*yaw_sign
                elif key=="a":
                    yaw-=0.05*yaw_sign
                elif key=="w":
                    pitch+=0.05*pitch_sign
                elif key=="s":
                    pitch-=0.05*pitch_sign
            
            
            theta1, theta2 = ik_2modules(90, pitch, yaw)
            if theta1 >= np.pi/2 or theta1<=-np.pi/2 or theta2>=np.pi/2 or theta2<=-np.pi/2:
                print("Out of Reach!")
                pitch = copy.copy(pitch_before)
                yaw = copy.copy(yaw_before)
                continue
            
            msg.position = prev_pose
            msg.position[1] = theta2
            msg.position[0] = theta1
            
            prev_pose[1] = copy.copy(theta2)
            prev_pose[0] = copy.copy(theta1)
            
            print("Joint angles :",[theta1, theta2])
            pub.publish(msg)
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

