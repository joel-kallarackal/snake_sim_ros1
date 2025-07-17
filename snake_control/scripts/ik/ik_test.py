import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])

def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]
    ])

def trans_x(a):
    return np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def fk_3r(theta1, theta2, theta3, a1, a2, a3):
    T1 = rot_z(theta1) @ trans_x(a1)
    T2 = rot_y(theta2) @ trans_x(a2)
    T3 = rot_z(theta3) @ trans_x(a3)
    T = T1 @ T2 @ T3
    return T

# Example usage
theta1, theta2, theta3 = np.radians([0, 0, 90])
a1 = a2 = a3 = 1.0

T = fk_3r(theta1, theta2, theta3, a1, a2, a3)
position = T[:3, 3]
print("End-effector position:", position)

# import numpy as np
# theta1 = np.linspace(-np.pi/2, np.pi/2, 20)
# theta2 = np.linspace(-np.pi/2, np.pi/2, 20)
# theta3 = np.linspace(-np.pi/2, np.pi/2, 20)

# x = []
# y = []
# z = []

# for t1 in theta1:
#     for t2 in theta2:
#         for t3 in theta3:
#             T = fk_3r(t1, t2, t3, a1, a2, a3)
#             position = T[:3, 3]
#             x.append(position[0])
#             y.append(position[1])
#             z.append(position[2])
        
# # Create 3D figure
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Scatter plot
# ax.scatter(x, y, z, c='blue', marker='o')

# # Labels (optional)
# ax.set_xlabel('X Axis')
# ax.set_ylabel('Y Axis')
# ax.set_zlabel('Z Axis')

# # Show plot
# plt.show()


###################################################################################################

import numpy as np

def ik_3r(x, y, z, a1, a2, a3):
    theta1 = np.arctan2(y, x)

    x_prime = np.sqrt(x**2 + y**2) - a1
    z_prime = z
    r = np.sqrt(x_prime**2 + z_prime**2)

    # Elbow-down solution
    cos_theta3 = (r**2 - a2**2 - a3**2) / (2 * a2 * a3)
    if abs(cos_theta3) > 1.0:
        raise ValueError("No IK solution: point out of reach")

    theta3 = np.arccos(cos_theta3)

    phi = np.arctan2(z_prime, x_prime)
    psi = np.arccos((a2**2 + r**2 - a3**2) / (2 * a2 * r))
    theta2 = phi - psi

    return theta1, theta2, theta3

print(ik_3r(2 ,1, 0, 1, 1, 1))