import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""
DH Parameters for three snake modules
      ------------------------------
      | theta |  d  |  a   | alpha |
|-----|-------|-----|------|-------|
|  1  |   0   |  0  |  l1  | -90   |
|  2  |   0   |  0  |  l2  | +90   |
|  3  |   0   |  0  |  l3  |  0    | 
"""

def T1(theta, a):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, 0, -s, a*c],
        [s, 0, c, a*s],
        [0,  -1, 0, 0],
        [0,  0, 0, 1]
    ])

def T2(theta, a):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s, a*c],
        [ s, 0, -c, a*s],
        [ 0, 1, 0, 0],
        [ 0, 0, 0, 1]
    ])

def T3(theta, a):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, a*c],
        [s, c, 0, a*s],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def fk(theta1, theta2, theta3, a1, a2, a3):
    T = T1(theta1, a1) @ T2(theta2, a2) @ T3(theta3, a3)
    return T

theta1, theta2, theta3 = np.radians([0, 0, 0])
a1 = a2 = a3 = 1.0

T = fk(theta1, theta2, theta3, a1, a2, a3)
position = T[:3, 3]
print("End-effector position:", position)

# Workspace Visualization

import numpy as np
theta1 = np.linspace(-np.pi, np.pi, 10)
theta2 = np.linspace(-np.pi, np.pi, 10)
theta3 = np.linspace(-np.pi, np.pi, 10)

x = []
y = []
z = []

for t1 in theta1:
    for t2 in theta2:
        for t3 in theta3:
            T = fk(t1, t2, t3, 1, 1, 1)
            position = T[:3, 3]
            x.append(position[0])
            y.append(position[1])
            z.append(position[2])
        
# Create 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
ax.scatter(x, y, z, c='blue', marker='o')

# Labels (optional)
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# Show plot
plt.show()