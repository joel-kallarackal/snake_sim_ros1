import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.stats import gaussian_kde
from collections import Counter

"""
DH Parameters for six snake modules
      ------------------------------
      | theta |  d  |  a   | alpha |
|-----|-------|-----|------|-------|
|  1  |   0   |  0  |  l6  | 90    |
|  2  |   0   |  0  |  l5  | 90    |
|  3  |   0   |  0  |  l4  | 90    |
|  4  |   0   |  0  |  l3  | 90    |
|  5  |   0   |  0  |  l2  | 90    |
|  6  |   0   |  0  |  l1  |  0    | 
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

a1 = a2 = a3 = a4 = a5 = a6 = 1.0

dh = [[0, 0, a6, np.pi/2],
      [0, 0, a5, np.pi/2],
      [0, 0, a4, np.pi/2],
      [0, 0, a3, np.pi/2],
      [0, 0, a2, np.pi/2],
      [0, 0, a1, 0]]

final_T = fk(dh)

position = final_T[:3, 3]
print("End-effector position:", position)

# Workspace Visualization

import numpy as np
theta = np.linspace(-np.pi/2, np.pi/2, 5)

x = []
y = []
z = []

# for t1 in theta:
#     for t2 in theta:
#         for t3 in theta:
#             for t4 in theta:
#                 for t5 in theta:
#                     for t6 in theta:
#                         dh[0][0] = t1
#                         dh[1][0] = t2
#                         dh[2][0] = t3
#                         dh[3][0] = t4
#                         dh[4][0] = t5
#                         dh[5][0] = t6
                        
#                         final_T = fk(dh)
#                         position = final_T[:3, 3]
#                         x.append(position[0])
#                         y.append(position[1])
#                         z.append(position[2])
                    

for t1 in theta:
    for t2 in theta:
        for t3 in theta:
            for t4 in theta:
                for t5 in theta:
                    dh[0][0] = t1
                    dh[1][0] = t2
                    dh[2][0] = t3
                    dh[3][0] = t4
                    dh[4][0] = t5
                    dh[5][0] = 0
                
                    final_T = fk(dh)
                    position = final_T[:3, 3]
                    x.append(position[0])
                    y.append(position[1])
                    z.append(position[2])
        
# # Create 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
# ax.scatter(x, y, z, s=1, alpha=0.5)

# # Labels (optional)
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# # Show plot
# plt.show()

# Scatter plot with density based on nearest neighbours
xyz = np.vstack([x, y, z])
kde = gaussian_kde(xyz)
densities = kde(xyz)

# Then plot with colors mapped to density
ax.scatter(x, y, z, c=densities, cmap='viridis', s=4)
plt.show()

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