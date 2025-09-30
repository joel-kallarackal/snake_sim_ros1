import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.stats import gaussian_kde
from collections import Counter

"""
DH Parameters for three snake modules
      ------------------------------
      | theta |  d  |  a   | alpha |
|-----|-------|-----|------|-------|
|  1  |   0   |  0  |  l3  | 90    |
|  2  |   0   |  0  |  l2  | 90    |
|  3  |   0   |  0  |  l1  |  0    | 
"""

def T(theta, d, a, alpha):
    c, s = np.cos(theta), np.sin(theta)
    c_alpha, s_alpha = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -c_alpha*s, -s_alpha*s, a*c],
        [s, c_alpha*c, -s_alpha*c, a*s],
        [0, s_alpha, c_alpha, d],
        [0, 0, 0, 1]
    ])

def fk(dh):
    final_T = T(dh[0][0], dh[0][1], dh[0][2], dh[0][3])
    for i in range(1,len(dh)):
        final_T = final_T @ T(dh[i][0], dh[i][1], dh[i][2], dh[i][3])
    return final_T

a1 = a2 = a3 = 1.0

dh = [[0, 0, a3, np.pi/2],
      [0, 0, a2, np.pi/2],
      [0, 0, a1, 0]]

final_T = fk(dh)

position = final_T[:3, 3]
print("End-effector position:", position)

# Workspace Visualization

import numpy as np
theta = np.linspace(-np.pi/2, np.pi/2, 7)

x = []
y = []
z = []

x_orient = []
y_orient = []
z_orient = []

for t1 in theta:
    for t2 in theta:
        for t3 in theta:
            dh[0][0] = t1
            dh[1][0] = t2
            dh[2][0] = t3
            
            final_T = fk(dh)
            position = final_T[:3, 3]
            
            x_axis = final_T[:3, 0]
            y_axis = final_T[:3, 1]
            z_axis = final_T[:3, 2]
            
            x.append(position[0])
            y.append(position[1])
            z.append(position[2])
            x_orient.append(x_axis)
            y_orient.append(y_axis)
            z_orient.append(z_axis)
        
# # Create 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
# ax.scatter(x, y, z, s=1, alpha=0.5)

# # Labels (optional)
# ax.set_xlabel('X Axis')
# ax.set_ylabel('Y Axis')
# ax.set_zlabel('Z Axis')

# # Show plot
# plt.show()

# Scatter plot with density based on nearest neighbours
# xyz = np.vstack([x, y, z])
# kde = gaussian_kde(xyz)
# densities = kde(xyz)

# # Then plot with colors mapped to density
# ax.scatter(x, y, z, c=densities, cmap='viridis', s=2)
# plt.show()

# Scatter plot of points with density based on duplicates
# positions = np.vstack((x, y, z)).T
# rounded_positions = np.round(positions, decimals=3)

# # --- Step 2: Count duplicates ---
# position_tuples = [tuple(p) for p in rounded_positions]
# counts = Counter(position_tuples)

# # --- Step 3: Extract unique positions and densities ---
# unique_positions = np.array(list(counts.keys()))
# densities = np.array([counts[pos] for pos in counts])

# # --- Step 4: Plot ---
# sc = ax.scatter(unique_positions[:, 0],
#                 unique_positions[:, 1],
#                 unique_positions[:, 2],
#                 c=densities,
#                 cmap='viridis',
#                 s=5)

# plt.title("Workspace Density Based on Duplicate Points")
# plt.colorbar(sc, label='Duplicate Count')
# plt.tight_layout()
# plt.show()

# Scatter plot with orientation
arrow_length = 0.03  # change as needed

for i in range(len(x)):
    pos = np.array([x[i], y[i], z[i]])
    
    # Orientation vectors
    x_vec = np.array(x_orient[i])
    y_vec = np.array(y_orient[i])
    z_vec = np.array(z_orient[i])
    
    vec = x_vec+y_vec+z_vec

    # Plot position
    # ax.scatter(*pos, color='k', s=2)

    # Plot orientation arrows
    # ax.quiver(*pos, *x_vec, color='r', length=arrow_length, normalize=True)
    # ax.quiver(*pos, *y_vec, color='g', length=arrow_length, normalize=True)
    # ax.quiver(*pos, *z_vec, color='b', length=arrow_length, normalize=True)
    
    ax.quiver(*np.array([0,0,0]), *vec, color='b', length=arrow_length, normalize=True)

# Set axes labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3 Modules : Orientations Acheived")
plt.show()
