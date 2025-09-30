import pybullet as p
import pybullet_data
import time

# Connect to physics server
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Path to your STL
stl_path = "/home/kallrax/biorobotics_lab/ros1_ws/src/snakelib_bullet/terrain/junglegym3.STL"

# Load convex mesh
collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName=stl_path,
    flags=p.GEOM_CONCAVE_INTERNAL_EDGE  # required for collision
)

visual_shape = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName=stl_path
)

body_id = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=collision_shape,
    baseVisualShapeIndex=visual_shape,
    basePosition=[0, 0, 1]
)

# Run sim
p.setGravity(0,0,-9.8)
while True:
    p.stepSimulation()
    time.sleep(1./240.)
