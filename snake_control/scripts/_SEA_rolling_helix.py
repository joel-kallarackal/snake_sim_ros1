#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from typing import Dict
from rolling_helix import rolling_helix
import math
from sympy import symbols, solve, Eq, sin as sym_sin
import time
import pybullet as p

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

k=0
i=0
j=0
initial_pose = [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
X = [0.04, 0.0, 0.0]
joint_angle=0
# -------------------- gait / helix parameters --------------------
beta_lat = 0
beta_dor = 0

# Spatial frequency (controls how tight the helix winds along the backbone)
f_s = 0
f_sfin = 2 * math.pi / 18  
# Temporal frequency for dynamic gait (after preset)
f_t_dynamic = -1  
# For preset helix freeze time dependence
f_t_preset = 0  
delta =  -math.pi / 2  # quadrature for 3D twisting

# Helix-related geometric parameters
l_m = 0.06604  # module length
D_m = 0.0508
D_pip = 0.0762
radius = (D_pip + D_m) / 2

# Solve for A from the pitch/radius relation
A = symbols('A')
pitch_expr = l_m / (((A / (2 * sym_sin(f_sfin))) ** 2 + 1) * f_sfin)
eq = Eq(A / (2 * sym_sin(f_sfin)) * pitch_expr, radius)
A_sol = solve(eq, A)
# filter real solutions
A_real = [sol for sol in A_sol if sol.is_real]
if not A_real:
    raise ValueError(f"No real solution for A: {A_sol}")
A_min = max(A_real)
pitch_val = pitch_expr.subs(A, A_min)
handedness = np.sign(float(A_min * f_sfin))

# Slightly inflate amplitude for cripping
A_val = float(A_min * 1.05)
A_lat, A_dor = 0, 0 
A_latfin = A_dorfin = A_val

print("==================================================================================")
print(f"A_min = {A_min}, pitch = {pitch_val}, radius = {radius}, handedness = {handedness}")
print(f"Using amplitudes: A_lat = A_dor = {A_val}")
print("==================================================================================")


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/snake/joint_commands', JointState, queue_size=10)
        rospy.init_node('controller', anonymous=True)
        t=0
        while not rospy.is_shutdown():
            msg = JointState()
            msg.name = ["SA001__MoJo",
                        "SA002__MoJo",
                        "SA003__MoJo",
                        "SA004__MoJo",
                        "SA005__MoJo",
                        "SA006__MoJo",
                        "SA007__MoJo",
                        "SA008__MoJo",
                        "SA009__MoJo",
                        "SA0010__MoJo",
                        "SA0011__MoJo",
                        "SA0012__MoJo",
                        "SA0013__MoJo",
                        "SA0014__MoJo",
                        "SA0015__MoJo",
                        "SA0016__MoJo",]

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
                                    
        
            # beta_even = 0
            # beta_odd = 0
            # A_even = 1.2
            # A_odd = 1.2
            # k=1
            # # A_even+=0.05
            # # A_odd+=0.05
            
            # delta_even = np.pi/2
            # delta_odd = 0
            # msg.position = [gait(get_param(i,beta_even,beta_odd),
            #                     get_param(i,A_even,A_odd),
            #                     k,
            #                     i,
            #                     0.5,
            #                     t,
            #                     get_param(i,delta_even,delta_odd)) for i in range(16)]
            
            if A_lat < A_latfin:
                A_lat += 0.001
                A_dor += 0.001
                pos = []
                for n in range(16):
                    if n % 4 == 0:
                        angle = beta_lat + A_lat * math.sin(-f_t_dynamic * t)
                    elif n % 4 == 1:
                        angle = beta_dor + A_dor * math.sin(-f_t_dynamic * t + delta)
                    elif n % 4 == 2:
                        angle = -beta_lat - A_lat * math.sin(-f_t_dynamic * t)
                    elif n % 4 == 3:
                        angle = beta_dor - A_dor * math.sin(-f_t_dynamic * t + delta)
                    pos.append(angle)
            elif f_s<f_sfin:
                f_s += 0.001
                pos = []
                for n in range(16):
                    if n % 4 == 0:
                        angle = beta_lat + A_lat * math.sin(f_s * n - f_t_dynamic * t)
                    elif n % 4 == 1:
                        angle = beta_dor + A_dor * math.sin(f_s * n - f_t_dynamic * t + delta)
                    elif n % 4 == 2:
                        angle = -beta_lat - A_lat * math.sin(f_s * n - f_t_dynamic * t)
                    elif n % 4 == 3:
                        angle = beta_dor - A_dor * math.sin(f_s * n - f_t_dynamic * t + delta)
                    pos.append(angle)
            else:
                pos = []
                for n in range(16):
                    if n % 4 == 0:
                        angle = beta_lat + A_lat * math.sin(f_s * n - f_t_dynamic * t)
                    elif n % 4 == 1:
                        angle = beta_dor + A_dor * math.sin(f_s * n - f_t_dynamic * t + delta)
                    elif n % 4 == 2:
                        angle = -beta_lat - A_lat * math.sin(f_s * n - f_t_dynamic * t)
                    elif n % 4 == 3:
                        angle = beta_dor - A_dor * math.sin(f_s * n - f_t_dynamic * t + delta)
                    pos.append(angle)
                
                
            
            msg.position = pos
            pub.publish(msg)
            
            t+=1/240
            time.sleep(1/240)
    except rospy.ROSInterruptException:
        pass

