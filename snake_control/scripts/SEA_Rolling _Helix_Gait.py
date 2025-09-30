import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np
from sympy import symbols, solve, Eq, sin as sym_sin
import matplotlib as plt

# -------------------- setup simulation --------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF('plane.urdf')

# load SEA Snake (adjust path if needed)
os.chdir('/home/kallrax/biorobotics_lab/ros1_ws/src/snakelib_description/SEA_snake')
start_pos = [0, 0, 0]
start_ori = p.getQuaternionFromEuler([0, math.pi/2, 0])
robot = p.loadURDF('SEA_snake.urdf', basePosition=start_pos, baseOrientation=start_ori, useFixedBase=False)

# ---------- T-junction pipe (two separate static cylinders) ----------
# orientations: cylinder axis is local Z
# main pipe along X: rotate Z -> X by +90° about Y
orn_main = p.getQuaternionFromEuler([0, 0, 0])
# branch along Y: rotate Z -> Y by -90° about X
orn_branch = p.getQuaternionFromEuler([-math.pi / 2, 0, 0])

# T-junction parameters
main_position = [0, 0, 0.5]
branch_position = [0, 0, 1]
main_length = 1   # along X
branch_length = 1  # along Y
pipe_radius = 0.028

# main pipe body
main_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=pipe_radius, height=main_length)
main_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=pipe_radius, length=main_length)
main_body = p.createMultiBody(
    baseMass= 0,
    baseCollisionShapeIndex=main_col,
    baseVisualShapeIndex=main_vis,
    basePosition=main_position,
    baseOrientation=orn_main,
)
p.changeVisualShape(main_body, -1, rgbaColor=[0.769, 0.557, 0.384, 1.0])

# branch pipe body
branch_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=pipe_radius, height=branch_length)
branch_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=pipe_radius, length=branch_length)
branch_body = p.createMultiBody(
    baseMass=0.0,
    baseCollisionShapeIndex=branch_col,
    baseVisualShapeIndex=branch_vis,
    basePosition=branch_position,
    baseOrientation=orn_branch,
)
p.changeVisualShape(branch_body, -1, rgbaColor=[0.769, 0.557, 0.384, 1.0])

# set lateral (sliding) friction
p.changeDynamics(main_body,   -1, lateralFriction=1)
p.changeDynamics(branch_body, -1, lateralFriction=1)

# # (optionally) set spinning and rolling friction for more realistic pipe‐snake contact
# p.changeDynamics(main_body,   -1, spinningFriction=1, rollingFriction=1)
# p.changeDynamics(branch_body, -1, spinningFriction=1, rollingFriction=1)

# # (optionally) set restitution (bounciness) — usually zero for pipes
p.changeDynamics(main_body,   -1, restitution=0.0)
p.changeDynamics(branch_body, -1, restitution=0.0)

# ------------------------------------------------------------

# gather revolute joints
revolute_joints = []
for i in range(p.getNumJoints(robot)):
    if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE:
        revolute_joints.append(i)
# reverse if original head/tail mapping required
revolute_joints.reverse()

# camera / gravity initial (will adjust gravity during preset)
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw= 50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])
# Temporarily disable gravity to prevent sagging during preset
p.setGravity(0, 0, 0)

# -------------------- gait / helix parameters --------------------
beta_lat = 0
beta_dor = 0

# Spatial frequency (controls how tight the helix winds along the backbone)
f_s = 2 * math.pi / 18  
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
pitch_expr = l_m / (((A / (2 * sym_sin(f_s))) ** 2 + 1) * f_s)
eq = Eq(A / (2 * sym_sin(f_s)) * pitch_expr, radius)
A_sol = solve(eq, A)
# filter real solutions
A_real = [sol for sol in A_sol if sol.is_real]
if not A_real:
    raise ValueError(f"No real solution for A: {A_sol}")
A_min = max(A_real)
pitch_val = pitch_expr.subs(A, A_min)
handedness = np.sign(float(A_min * f_s))

# Slightly inflate amplitude for cripping
A_val = float(A_min * 1.05)
A_lat = A_dor = A_val

print("==================================================================================")
print(f"A_min = {A_min}, pitch = {pitch_val}, radius = {radius}, handedness = {handedness}")
print(f"Using amplitudes: A_lat = A_dor = {A_val}")
print("==================================================================================")

# -------------------- preset snake as static helix around main pipe --------------------


# Place snake base offset in Y so it starts wrapped outside the main pipe
base_pos = [0, 0, 0.1]
climb_ori = p.getQuaternionFromEuler([1.5*math.pi/2, 2.8*math.pi/4 ,0])
p.resetBasePositionAndOrientation(robot, base_pos, climb_ori)

# Compute and apply static helix-like joint angles (using frozen temporal term)
static_angles = {}
for n, joint_id in enumerate(revolute_joints):
    if n % 4 == 0:
        angle = beta_lat + A_lat * math.sin(f_s * n + f_t_preset * 0)
    elif n % 4 == 1:
        angle = beta_dor + A_dor * math.sin(f_s * n + f_t_preset * 0 + delta)
    elif n % 4 == 2:
        angle = -beta_lat - A_lat * math.sin(f_s * n + f_t_preset * 0)
    elif n % 4 == 3:
        angle = beta_dor - A_dor * math.sin(f_s * n + f_t_preset * 0 + delta)
    static_angles[joint_id] = angle
    p.resetJointState(robot, joint_id, targetValue=angle)

# Bake in the preset shape by stepping a few times
for _ in range(50):
    p.stepSimulation()
    time.sleep(1. / 120)

# Re-enable gravity for dynamics
p.setGravity(0, 0, -9.81)

# -------------------- dynamic control parameters --------------------
force = 20
start_time = time.time()
step1_angles = {}
ind = 0

# -------------------- simulation loop: start dynamic traveling-wave gait --------------------
while time.time() - start_time < 10:
    t = time.time() - start_time
    for n, joint_id in enumerate(revolute_joints):
        if n % 4 == 0:
            angle = beta_lat + A_lat * math.sin(f_s * n + f_t_dynamic * t)
        elif n % 4 == 1:
            angle = beta_dor + A_dor * math.sin(f_s * n + f_t_dynamic * t + delta)
        elif n % 4 == 2:
            angle = -beta_lat - A_lat * math.sin(f_s * n + f_t_dynamic * t)
        elif n % 4 == 3:
            angle = beta_dor - A_dor * math.sin(f_s * n + f_t_dynamic * t + delta)

        if ind == 0:
            step1_angles[joint_id] = [angle]
        else:
            step1_angles[joint_id].append(angle)
        
        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=force
        )
    ind += 1
    p.stepSimulation()
    time.sleep(1. / 240)