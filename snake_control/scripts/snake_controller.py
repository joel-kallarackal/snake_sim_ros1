#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from typing import Dict
from rolling_helix import rolling_helix

JOINT_TOLERANCE = 0.15  # tolerance for joint angles
MIN_JOINT_ANGLE = -np.pi / 2.0 + JOINT_TOLERANCE
MAX_JOINT_ANGLE = np.pi / 2.0 - JOINT_TOLERANCE
dt = 0.025

import sys
import termios
import tty
import copy

def create_arc():
    # Find head start orientation
    head_acc_x, head_acc_y = 0, 9.6
    pole_direction = 1
    beta = 0.3
    # Choose joints that have most of their motion along ground.
    module_offset = 1 if abs(head_acc_x) > abs(head_acc_y) else 0

    inverted_module = -1
    if abs(head_acc_x) > abs(head_acc_y):
        if head_acc_x > 0:
            inverted_module *= -1
    else:
        if head_acc_y > 0:
            inverted_module *= -1

    # Create arc in the given direction.
    arc_angles = np.zeros(16)
    # Account for flipping in hardware
    module_flip = np.ones(len(arc_angles[module_offset::2]))
    module_flip[1::2] *= -1
    arc_angles[module_offset::2] = inverted_module * pole_direction * beta * module_flip

    return arc_angles

def head_look(t: float = 0, current_angles: np.ndarray = None, params: Dict = None):
    """
    Args:
        t: current robot time
        current angles: latest joint angles readings
        params: update (if any) in parameters
    """
    n_headlook_modules = 2  # number of modules in the headlook group.

    current_gait = "head_look"
    # Update the current parameters if params is not empty
    params = {} if params is None else params
    x_state, y_state = 5,5

    # Use accelerations to correct direction of rotation.
    head_acc = [-2,-1]
    before_head_vel = y_state if abs(head_acc[1]) > abs(head_acc[0]) else x_state
    major_comp_sign = np.sign(head_acc[np.argmax(np.abs(head_acc))])
    before_head_vel *= major_comp_sign
    head_vel = x_state if abs(head_acc[1]) > abs(head_acc[0]) else y_state
    head_vel *= -np.sign(head_acc[np.argmax(np.abs(head_acc))])
    if abs(head_acc[1]) > abs(head_acc[0]):
        head_vel *= -1

    vel = np.array([head_vel, before_head_vel])

    # Slice head group joint angles form all joint angles.
    headlook_group_current_angles = current_angles[:n_headlook_modules]

    headlook_group_target_angles = headlook_group_current_angles + (vel * dt)
    headlook_group_target_angles = np.clip(headlook_group_target_angles, MIN_JOINT_ANGLE, MAX_JOINT_ANGLE)
    target_angles = np.concatenate((headlook_group_target_angles, current_angles[n_headlook_modules:]))

    return target_angles

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

def project_to_torus(x, y, z, l1, l2):
    # Step 1: Azimuthal angle
    theta1 = np.arctan2(y, x)
    R = np.sqrt(x**2 + y**2)

    # Step 2: Compute elevation angle φ
    phi = np.arctan2(z, R - l1)

    # Step 3: Compute projected point on torus surface
    x_proj = (l1 + l2 * np.cos(phi)) * np.cos(theta1)
    y_proj = (l1 + l2 * np.cos(phi)) * np.sin(theta1)
    z_proj = l2 * np.sin(phi)

    return x_proj, y_proj, z_proj


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
X = [0.04, 0.0, 0.0]
joint_angle=0
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
            # beta_even = 0
            # beta_odd = 1
            # A_even = 0.3
            # A_odd = 0.3
            # delta_even = 0
            # delta_odd = 1
            
            # Test
            # msg.position = [gait(0,1,1,i,1,t,0) for i in range(16)]
            
            # Lateral Undulation
            # beta_even = 0
            # beta_odd = 0
            # A_even = 1.5
            # A_odd = 0
            # delta_even = 0
            # delta_odd = 0
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      1,
            #                      i,
            #                      0.5,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            # Linear Progression
            # beta_even = 0.4
            # beta_odd = 0
            # A_even = 0
            # A_odd = 0.1
            # delta_even = 0
            # delta_odd = 0
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      1,
            #                      i,
            #                      1,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            # Rolling
            # beta_even = 0
            # beta_odd = 0
            # A_even = 0.1
            # A_odd = 0.1
            # delta_even = 0
            # delta_odd = 0
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      1,
            #                      i,
            #                      1,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
           
            # Double Linear Progression
            # beta_even = 0.8
            # beta_odd = 0.4
            # A_even = 0.1
            # A_odd = 0.1
            # delta_even = 0
            # delta_odd = 0
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      1,
            #                      i,
            #                      1,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            # Rolling
            # beta_even = 0
            # beta_odd = 0
            # A_even = 1
            # A_odd = 1
            # delta_even = 0
            # delta_odd = 0
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      1,
            #                      i,
            #                      0.5,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
                                    
            
            # Compress and release
             
             
            # key = get_key()
            # if key=="q":
            #     joint_angle+=0.05
            # elif key=="a": 
            #     joint_angle-=0.05
            # else:
            #     pass
            # beta_even = 0
            # beta_odd = 0
            # A_even = 0 if t%2==0 else -1
            # A_odd = 0
            # delta_even = 0
            # delta_odd = 0
            # msg.position = np.concatenate((np.array([0, 0, joint_angle, 0]),np.array([gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      0,
            #                      i,
            #                      0,
            #                      t,
            #                      get_param(i,delta_even+(i/2)*(np.pi/2),delta_odd)) for i in range(12)])))
            # pub.publish(msg)
            
            # Fixed shape, since time not varying
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      1,
            #                      i,
            #                      0,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            # Sidewinding
            beta_even = 0
            beta_odd = 0
            A_even = 0.8
            A_odd = 0.8
            delta_even = 0
            delta_odd = np.pi/4
            msg.position = [gait(get_param(i,beta_even,beta_odd),
                                 get_param(i,A_even,A_odd),
                                 0.5,
                                 i,
                                 0.7,
                                 t,
                                 get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            pub.publish(msg)
            
            # Snake forms and arc and rolls
            # beta_even = 0
            # beta_odd = 0
            # A_even = 0.5
            # A_odd = 0.5
            # delta_even = 0
            # delta_odd = np.pi/2
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      0,
            #                      i,
            #                      0.5,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            # Snake forms arc and rolls, then arc radius decreases as time goes
            # beta_even = 0
            # beta_odd = 0
            # # A_even = 0.5
            # # A_odd = 0.5
            # A_even+=0.05
            # A_odd+=0.05
            # delta_even = 0
            # delta_odd = np.pi/2
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      0,
            #                      i,
            #                      0.5,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
            # Transition from rolling to rolling helix
            
            
            # if A_even<1.2 and execute_pole_climb:
            #     beta_even = 0
            #     beta_odd = 0
            #     # A_even = 0.5
            #     # A_odd = 0.5
            #     A_even+=0.05
            #     A_odd+=0.05
            #     delta_even = np.pi/2
            #     delta_odd = 0
            #     msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                         get_param(i,A_even,A_odd),
            #                         0,
            #                         i,
            #                         0.5,
            #                         t,
            #                         get_param(i,delta_even,delta_odd)) for i in range(16)]
            #     if not pole_climb_done:
            #         pub.publish(msg)
            # elif execute_pole_climb:
            #     beta_even = 0
            #     beta_odd = 0
            #     delta_even = np.pi/2
            #     delta_odd = 0
            #     k+=0.01
            #     msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                         get_param(i,A_even,A_odd),
            #                         k,
            #                         i,
            #                         0.5,
            #                         t,
            #                         get_param(i,delta_even,delta_odd)) for i in range(16)]
            #     j+=1
            #     if j==10:
            #         pole_climb_done=True
            #         initial_pose = msg.position
            #         # FK to get inital pose
            #         l1 = 0.02
            #         l2 = 0.02
            #         theta1 = initial_pose[1]
            #         theta2 = initial_pose[0]
            #         x = (l1 + l2 * np.cos(theta2)) * np.cos(theta1)
            #         y = (l1 + l2 * np.cos(theta2)) * np.sin(theta1)
            #         z = l2 * np.sin(theta2)
            #         X = [x, y, z]
            #         print(msg.position)
                
            #     if not pole_climb_done:
            #         pub.publish(msg)
                
            # else:
            #     i+=1
            #     if i==4:
            #         execute_pole_climb=True

            #     # Make a roll to head lift orientation
            #     beta_even = 0
            #     beta_odd = 0
            #     # A_even = 0.5
            #     # A_odd = 0.5
            #     A_even=0.05
            #     A_odd=0.05
            #     delta_even = 0
            #     delta_odd = np.pi/2
            #     msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                         get_param(i,A_even,A_odd),
            #                         0,
            #                         i,
            #                         0.5,
            #                         t,
            #                         get_param(i,delta_even,delta_odd)) for i in range(16)]
            #     pub.publish(msg)
                                
            
                
            # Test
            # beta_even = 0 # 0.4
            # beta_odd = 0
            # A_even = 0
            # A_odd = 1
            # delta_even = 0
            # delta_odd = 0
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                      get_param(i,A_even,A_odd),
            #                      1.4,
            #                      i,
            #                      0,
            #                      t,
            #                      get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            
            # msg.position = [0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0]
            # msg.position = [-0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0]
            
            # msg.position = [-0.425, 0.125, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0, -0.3, 0.0, 0.3, 0.0]
            # msg.position = [ 0.0, -0.49991211, -0.01874561, 0.49920919, 0.03746485, -0.49780434, -0.05613142, 0.49569954, 0.07471907, -0.49289774, -0.09320165, 0.48940288, 0.11155318, -0.48521988, -0.12974786, 0.48035462]
            
            # Head Look
            # current_angles = head_look(0,current_angles,{})
            # msg.position = current_angles
            
            # msg.position, gait_params = rolling_helix(t, gait_params)
            
            # msg.position = create_arc()
            
            ############################################################################################
            # IK Keyboard control 
            # pole_climb_done=True
            # if pole_climb_done:
            #     l1=0.02
            #     l2=0.02
            #     X_before = copy.copy(X)
            #     key = get_key()
            #     if key=="q":
            #         X[0]+=0.005
            #     elif key=="a":
            #         X[0]-=0.005
            #     elif key=="w":
            #         X[1]+=0.005
            #     elif key=="s":
            #         X[1]-=0.005
            #     elif key=="e":
            #         X[2]+=0.005
            #     elif key=="d":
            #         X[2]-=0.005
                
            #     try:
            #         x ,y, z = project_to_torus(X[0], X[1], X[2], l1, l2)
            #         # X = [x, y, z]
            #         theta1 = np.arctan2(X[1],X[0])
            #         theta2 = np.arctan2(X[2], np.sqrt(X[0]**2 + X[1]**2)-l1)
                    
            #         if theta1>=np.pi/2 or theta1<=-np.pi/2 or theta2>=np.pi/2 or theta2<=-np.pi/2:
            #             X = X_before
            #         else:
            #             print(x,y,z)
            #             initial_pose[0] = theta2
            #             initial_pose[1] = theta1
            #             msg.position = initial_pose
            #             pub.publish(msg)

            #     except:
            #         print("Out of reach!")
            #         X = copy.copy(X_before)
            ##########################################################################################
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

