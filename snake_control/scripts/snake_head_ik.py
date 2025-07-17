#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from typing import Dict
from rolling_helix import rolling_helix
import sys
import termios
import tty
import copy

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch


def gait(beta, A, k, n, w, t, delta):
    return (beta + A*np.sin(k*n - w*t + delta)) * (-1) ** np.floor(n / 2)

def get_param(i,even,odd):
    # return (1-i%2)*even + (i%2)*odd
    return (1-i%2)*even + (i%2)*odd

# Using ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="link1",
      origin_translation=[0,0,0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="link2",
      origin_translation=[1, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="link3",
      origin_translation=[1, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="head",
      origin_translation=[1, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    )
])

current_angles = [-0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0]
gait_params = {
    "beta_even": 0.0,
    "beta_odd": 0.0,
    "A_even": 0.2,
    "A_odd": 0.2,
    "wS_even": 1.2,
    "wS_odd": 1.2,
    "wT_even": 1.75,
    "wT_odd": 1.75,
    "delta": -1.57079632679,
    "tightness": 0.4,
    "pole_direction":1}

A_even = 0
A_odd = 0
k=0
i=0
j=0
execute_pole_climb = False
pole_climb_done = False
initial_pose = [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
X = [3.0, 0.0, 0.0]
theta, phi, r, R  = 0, 0, 2, 1 
theta1_prev, theta2_prev, theta3_prev = 0.0, 0.0, 0.0
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
        msg.position = [1.1274298724979068, 0.5214787464862762, -1.0233078625570493, -0.7257993868875412, 0.878389797370735, 0.9011846961260116, -0.6984531028825454, -1.040642615382698, 0.490671287278658, 1.1386133975017492, -0.26332795585375135, -1.1911912565647602, 0.025486569808375298, 1.1962800792499158, 0.21337088534773876, -1.1536769902554687]
        initial_pose = [1.1274298724979068, 0.5214787464862762, -1.0233078625570493, -0.7257993868875412, 0.878389797370735, 0.9011846961260116, -0.6984531028825454, -1.040642615382698, 0.490671287278658, 1.1386133975017492, -0.26332795585375135, -1.1911912565647602, 0.025486569808375298, 1.1962800792499158, 0.21337088534773876, -1.1536769902554687]
        
        # msg.position = [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # initial_pose = [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        X = chain.forward_kinematics([0,initial_pose[2],initial_pose[1],initial_pose[0],0])[:3, 3]
        r = np.sqrt((np.sqrt(X[0]**2+X[1]**2)-R)**2+X[2]**2)
        theta = np.arcsin(X[2]/r)
        phi = np.arctan2(X[1],X[0])
        theta1_prev, theta2_prev, theta3_prev = initial_pose[0],initial_pose[1],initial_pose[2]
        theta1, theta2, theta3 = initial_pose[0],initial_pose[1],initial_pose[2]
        print(r, theta, phi)
        
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
                        "reu_joint_out2in_15",]
            t+=1
            
            ############################################################################################
            # For Three Module Snake
            # IK Keyboard control    
            # pole_climb_done=True
            # if pole_climb_done:
            #     X_before = copy.copy(X)
            #     key = get_key()
            #     if key=="q":
            #         X[0]+=0.05
            #     elif key=="a":
            #         X[0]-=0.05
            #     elif key=="w":
            #         X[1]+=0.05
            #     elif key=="s":
            #         X[1]-=0.05
            #     elif key=="e":
            #         X[2]+=0.05
            #     elif key=="d":
            #         X[2]-=0.05
                
            #     try:
            #         print("X :", X)
                    
            #         if X[0]>=2.5 or X[0]<=1 or X[1]>=1 or X[1]<=-1 or X[2]>=1 or X[2]<=-1:
            #             continue
                    
            #         theta1, theta2, theta3 = chain.inverse_kinematics(X)[1:4]
                    
            #         print("Joint angles :",[theta1, theta2, theta3])
                    
            #         if theta1>=np.pi/2 or theta1<=-np.pi/2 or theta2>=np.pi/2 or theta2<=-np.pi/2 or theta3>=np.pi/2 or theta3<=-np.pi/2:
            #             X = X_before
            #             print(theta1, theta2, theta3)
            #             print("Out of reach!")
            #         else:
            #             initial_pose[0] = theta3
            #             initial_pose[1] = theta2
            #             initial_pose[2] = theta1
                        
            #             msg.position = initial_pose
                        
            #             print(theta1, theta2, theta3)
            #             pub.publish(msg)

            #     except:
            #         print("Exception, Out of reach!")
            #         X = copy.copy(X_before)
            ##########################################################################################
            # For Three Snake Modules, Method 2
            # IK Keyboard control 
            pole_climb_done=True
            if pole_climb_done:
                X_before = copy.copy(X)
                theta_before = copy.copy(theta)
                phi_before = copy.copy(phi)
                
                r_before = copy.copy(r)
                key = get_key()
                if key=="q":
                    theta+=0.05
                elif key=="a":
                    theta-=0.05
                elif key=="w":
                    phi+=0.05
                elif key=="s":
                    phi-=0.05
                elif key=="e":
                    r+=0.01
                elif key=="d":
                    r-=0.01
                    
                # try:
                R = 1
                if theta > np.pi/2 or theta<-np.pi/2 or phi>np.pi/2 or phi<-np.pi/2 or r<np.sqrt(2) or r>2:
                    print("Oops! Out of the Workspace")
                    theta = copy.copy(theta_before)
                    phi = copy.copy(phi_before)
                    r = copy.copy(r_before)
                    continue
                l = 1
                x = (R + r*np.cos(theta))*np.cos(phi)
                y = (R + r*np.cos(theta))*np.sin(phi)
                z = r*np.sin(theta)
                
                print(f"x : {x}\ny : {y}\nz : {z}\ntheta : {theta}\nphi : {phi}\nr : {r}")
                
                # print("x,y,x prev: ",x,y,z)
                # print("x,y,x : ",x,y,z)
                
                theta1, theta2, theta3 = chain.inverse_kinematics([x, y, z], initial_position=[0, theta1_prev, theta2_prev, theta3_prev, 0])[1:4]
                theta1_prev, theta2_prev, theta3_prev = copy.copy(theta1), copy.copy(theta2), copy.copy(theta3)
                
                print("############################")
                print(theta1, theta1_prev, theta2, theta2_prev, theta3, theta3_prev)
                print("#############################")
                theta1, theta2, theta3 = np.array([theta1, theta2, theta3]) + np.clip(np.array([theta1, theta2, theta3])-np.array([theta1_prev, theta2_prev, theta3_prev]), -0.2, 0.2)
                
                T = chain.forward_kinematics([0,theta1,theta2,theta3,0])
                position = T[:3, 3]
                print(f"Acheived Position : {position[0]}, {position[1]}, {position[2]}")
                
                # print(theta1-theta1_prev)
                # if abs(theta1-theta1_prev)>0.3 or abs(theta2-theta2_prev)>0.3 or abs(theta3-theta3_prev)>0.3:
                #     print("Large change in angle")
                #     theta = theta_before+0.001
                #     phi = phi_before+0.001
                #     r=r_before+0.001
                #     continue
                
                if theta1>=np.pi/2 or theta1<=-np.pi/2 or theta2>=np.pi/2 or theta2<=-np.pi/2 or theta3>=np.pi/2 or theta3<=-np.pi/2:
                    X = X_before
                    print(theta1, theta2, theta3)
                    print("Out of reach!")
                    continue
                else:
                    initial_pose[0] = theta3
                    initial_pose[1] = theta2
                    initial_pose[2] = theta1
                    
                    msg.position = initial_pose
                    
                    print(f"Joint angles :\ntheta1 : {theta1}\ntheta2 : {theta2}\ntheta3 : {theta3}")
                    pub.publish(msg)

                # except Exception as e:
                    # print("Exception")
                    # print(e)
                    # X = copy.copy(X_before)
            
            ##########################################################################################
            # For Two Snake Modules
            # IK Keyboard control 
            # pole_climb_done=True
            # if pole_climb_done:
            #     X_before = copy.copy(X)
            #     key = get_key()
            #     if key=="q":
            #         theta+=0.1
            #     elif key=="a":
            #         theta-=0.1
            #     elif key=="w":
            #         phi+=0.1
            #     elif key=="s":
            #         phi-=0.1
                
            #     try:
                    
            #         if theta >= np.pi/2 or theta<=-np.pi/2 or phi>=np.pi/2 or phi<=-np.pi/2:
            #             continue
            #         r, R = 1,1
            #         l = 1
            #         x = (R + r*np.cos(theta))*np.cos(phi)
            #         y = (R + r*np.cos(theta))*np.sin(phi)
            #         z = r*np.sin(theta)
                    
            #         theta1 = np.arctan2(y,x)
            #         theta2 = np.arcsin(z/l)
                    
            #         print("Joint angles :",[theta1, theta2])
                    
            #         if theta1>=np.pi/2 or theta1<=-np.pi/2 or theta2>=np.pi/2 or theta2<=-np.pi/2:
            #             X = X_before
            #             print(theta1, theta2)
            #             print("Out of reach!")
            #         else:
            #             initial_pose[0] = theta2
            #             initial_pose[1] = theta1
                        
            #             msg.position = initial_pose
                        
            #             print(theta1, theta2)
            #             pub.publish(msg)

            #     except:
            #         print("Exception, Out of reach!")
            #         X = copy.copy(X_before)
            
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

