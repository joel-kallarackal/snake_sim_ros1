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

# import numpy as np
# import matplotlib.pylab as plt
# theta1 = np.linspace(-np.pi/2, np.pi/2, 20)
# theta2 = np.linspace(-np.pi/2, np.pi/2, 20)
# theta3 = np.linspace(-np.pi/2, np.pi/2, 20)

# x = []
# y = []
# z = []

# for t1 in theta1:
#     for t2 in theta2:
#         for t3 in theta3:
#             T = chain.forward_kinematics([0,t1,t2,t3,0])
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


import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

chain.plot(chain.inverse_kinematics([2, 1, 0]), ax)
matplotlib.pyplot.show()