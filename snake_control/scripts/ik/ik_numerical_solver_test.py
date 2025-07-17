from RoboKinematics import CreateKinematicModel
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define Denavit-Hartenberg (DH) parameters for a 2-joint robot
dh_params = [
    {"frame_name": "link1", "joint_type": "r", "link_length": 1.0, "twist": 90, "offset": 0, "theta": 0},
    {"frame_name": "link2", "joint_type": "r", "link_length": 1.0, "twist": 0, "offset": 0, "theta": 0}
]

# Create the kinematic model
robot = CreateKinematicModel(dh_params, robot_name="2DOF Robot")

# Perform forward kinematics
joint_angles = [0, 90]

robot.f_kin(joint_angles)
transformation_matrices = robot.get_transforms(2, real=True)
jacobian = robot.jacobian()

# print(transformation_matrices,'\n')
# print(jacobian)

position = transformation_matrices[:3, 3]
orientation = transformation_matrices[:3, :3]

print(position)

############################################################################################

# import numpy as np
# theta1 = np.linspace(-np.pi/2, np.pi/2, 25)
# theta2 = np.linspace(-np.pi/2, np.pi/2, 25)

# x = []
# y = []
# z = []

# for t1 in theta1:
#     for t2 in theta2:
#         robot.f_kin([t1,t2])
#         transformation_matrices = robot.get_transforms(2, real=True)
#         position = transformation_matrices[:3, 3]
#         x.append(position[0])
#         y.append(position[1])
#         z.append(position[2])
        
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


########################################################################################################

# Define Denavit-Hartenberg (DH) parameters for a 3-joint robot
dh_params = [
    {"frame_name": "link1", "joint_type": "r", "link_length": 1, "twist": -90, "offset": 0, "theta": 0},
    {"frame_name": "link2", "joint_type": "r", "link_length": 1, "twist": 90, "offset": 0, "theta": 0},
    {"frame_name": "link3", "joint_type": "r", "link_length": 1, "twist": 0, "offset": 0, "theta": 0}
]

# Create the kinematic model
robot = CreateKinematicModel(dh_params, robot_name="3DOF Robot")

# Perform forward kinematics
joint_angles = [0, 0, 0]

robot.f_kin(joint_angles)
transformation_matrices = robot.get_transforms(3, real=True)
jacobian = robot.jacobian()

# # print(transformation_matrices,'\n')
# # print(jacobian)

# position = transformation_matrices[:3, 3]
# orientation = transformation_matrices[:3, :3]

# print(f"Angles : {joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}")
# print("FK result :",position)

# Perform inverse kinematics
t = [1, 1, 1]
t = robot.SE3(robot.get_transforms(3), merge_res=True)
joint_angles = robot.i_kin([1, 2, 0, 0, 0, 0])

# print("Cartesian position :", t)
print("IK result :", joint_angles)


#########################################################################################################

import numpy as np
theta1 = np.linspace(-np.pi/2, np.pi/2, 30)
theta2 = np.linspace(-np.pi/2, np.pi/2, 30)
theta3 = np.linspace(-np.pi/2, np.pi/2, 30)

x = []
y = []
z = []

for t1 in theta1:
    for t2 in theta2:
        for t3 in theta3:
            robot.f_kin([t1,t2,t3])
            transformation_matrices = robot.get_transforms(3, real=True)
            position = transformation_matrices[:3, 3]
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

################################################################################################

# # Define Denavit-Hartenberg (DH) parameters for a 4-joint robot
# dh_params = [
#     {"frame_name": "link1", "joint_type": "r", "link_length": 1, "twist": 90, "offset": 0, "theta": 0},
#     {"frame_name": "link2", "joint_type": "r", "link_length": 1, "twist": -90, "offset": 0, "theta": 0},
#     {"frame_name": "link3", "joint_type": "r", "link_length": 1, "twist": 90, "offset": 0, "theta": 0},
#     {"frame_name": "link4", "joint_type": "r", "link_length": 1, "twist": 0, "offset": 0, "theta": 0}
# ]

# # Create the kinematic model
# robot = CreateKinematicModel(dh_params, robot_name="4DOF Robot")

# # Perform forward kinematics
# joint_angles = [0, 0, 0, 0]

# robot.f_kin(joint_angles)
# transformation_matrices = robot.get_transforms(4, real=True)
# jacobian = robot.jacobian()

# # print(transformation_matrices,'\n')
# # print(jacobian)

# position = transformation_matrices[:3, 3]
# orientation = transformation_matrices[:3, :3]

# print(f"Angles : {joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}")
# print("FK result :",position)

# # Perform inverse kinematics
# t = [1, 1, 1]
# t = robot.SE3(robot.get_transforms(3), merge_res=True)
# joint_angles = robot.i_kin([1, 2, 0, 0, 0, 0])

# print("Cartesian position :", t)
# print("IK result :", joint_angles)


# #########################################################################################################

# import numpy as np
# theta1 = np.linspace(-np.pi/2, np.pi/2, 15)
# theta2 = np.linspace(-np.pi/2, np.pi/2, 15)
# theta3 = np.linspace(-np.pi/2, np.pi/2, 15)
# theta4 = np.linspace(-np.pi/2, np.pi/2, 15)

# x = []
# y = []
# z = []

# for t1 in theta1:
#     for t2 in theta2:
#         for t3 in theta3:
#             for t4 in theta4:
#                 robot.f_kin([t1,t2,t3,t4])
#                 transformation_matrices = robot.get_transforms(4, real=True)
#                 position = transformation_matrices[:3, 3]
#                 x.append(position[0])
#                 y.append(position[1])
#                 z.append(position[2])
        
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


