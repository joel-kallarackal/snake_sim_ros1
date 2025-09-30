import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np
from sympy import symbols, solve, Eq, sin as sym_sin

# -------------------- setup simulation --------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF('plane.urdf')

# load SEA Snake (adjust path if needed)
os.chdir('/home/harry/snake_robot_simulation/2D_Jamming_Concept/snakelib_description/SEA_snake')
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
main_position = [0, 0, 0.2]
branch_position = [0, 0, 0.4]
main_length = 0.4   # along X
branch_length = 1.5  # along Y
main_radius = 0.028
branch_radius = 0.028

# main pipe body
main_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=main_radius, height=main_length)
main_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=main_radius, length=main_length)
main_body = p.createMultiBody(
    baseMass= 0,
    baseCollisionShapeIndex=main_col,
    baseVisualShapeIndex=main_vis,
    basePosition=main_position,
    baseOrientation=orn_main,
)
p.changeVisualShape(main_body, -1, rgbaColor=[0.2, 0.6, 0.8, 1.0])

# branch pipe body
branch_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=branch_radius, height=branch_length)
branch_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=branch_radius, length=branch_length)
branch_body = p.createMultiBody(
    baseMass=0.0,
    baseCollisionShapeIndex=branch_col,
    baseVisualShapeIndex=branch_vis,
    basePosition=branch_position,
    baseOrientation=orn_branch,
)

# set lateral (sliding) friction
p.changeDynamics(main_body,   -1, lateralFriction=2)
p.changeDynamics(branch_body, -1, lateralFriction=2)

# # (optionally) set spinning and rolling friction for more realistic pipe‐snake contact
# p.changeDynamics(main_body,   -1, spinningFriction=1, rollingFriction=1)
# p.changeDynamics(branch_body, -1, spinningFriction=1, rollingFriction=1)

# # (optionally) set restitution (bounciness) — usually zero for pipes
p.changeDynamics(main_body,   -1, restitution=0.0)
p.changeDynamics(branch_body, -1, restitution=0.0)

p.changeVisualShape(branch_body, -1, rgbaColor=[0.2, 0.6, 0.8, 1.0])
# ------------------------------------------------------------

# gather revolute joints
revolute_joints = []
for i in range(p.getNumJoints(robot)):
    if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE:
        revolute_joints.append(i)
# reverse if original head/tail mapping required
# revolute_joints.reverse()

# camera / gravity initial (will adjust gravity during preset)
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw= 50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])

# -------------------- gait / helix parameters --------------------
beta_lat = 0
beta_dor = 0

# Spatial frequency (controls how tight the helix winds along the backbone)
f_s = 2 * math.pi / 18  
# Temporal frequency for dynamic gait (after preset)
f_t = 1  
# For preset helix freeze time dependence
f_t_preset = 0  
delta = math.pi / 2  # quadrature for 3D twisting

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
A_val = float(A_min * 1.1)
A_lat = A_dor = A_val

# Set the pulse parameters
k_r = 10 # radius of the helix in the pulse
k_p = 0.75 # pitch of the helix in the pulse
f_s_p = (k_p * pitch_val * l_m) / ((k_r * radius)**2 + (k_p * pitch_val)**2)
A_p = 2 * ((k_r * radius) / (k_p * pitch_val)) * math.sin(f_s_p)

# Set the windowed parameters
d_w = -handedness*(math.pi/2) # offsetbetween head and tail windows
n_b_u = 5 # user input module, indicating the T-transition direction
v_n_t = l_m/(math.pi*D_m*abs(f_t)) # velocity of transition location change
n_b = (3*math.pi/4)*((D_pip+D_m)/l_m)
n_t = 0 # initialize transition location

k_low = 0.4 # unwrapped amplitude
m_w = 5 # amplitude increase around n = n_b



print("==================================================================================")
print(f"A_min = {A_min}, pitch = {pitch_val}, radius = {radius}, handedness = {handedness}")
print(f"Using amplitudes: A_lat = A_dor = {A_val} ")
print("==================================================================================")

# -------------------- preset snake as static helix around main pipe --------------------
# Temporarily disable gravity to prevent sagging during preset
p.setGravity(0, 0, 0)

# Place snake base offset in Y so it starts wrapped outside the main pipe
base_pos = [-0.02, 0.04, 0]
climb_ori = p.getQuaternionFromEuler([0*math.pi/2, 1*math.pi/4 ,0])
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
        angle = -beta_dor - A_dor * math.sin(f_s * n + f_t_preset * 0 + delta)
    static_angles[joint_id] = angle
    p.resetJointState(robot, joint_id, targetValue=angle)

# Bake in the preset shape by stepping a few times
#while True: #
for _ in range(50):
    p.stepSimulation()
    time.sleep(1. / 120)

# Enable gravity for dynamics
p.setGravity(0, 0, -9.81)

# -------------------- dynamic control parameters --------------------
force = 20
n_s = 15 # Initiate the pulse center
d_t = 1./120 # Time step
v_ns = 1 # Pulse velocity
m = 30 # Slope of the exponential function
start_time = time.time()
d_T_s = 120

# -------------------- simulation loop: start dynamic traveling-wave gait --------------------
if abs(n_b - n_b_u):
    t = time.time() - start_time
    while t < d_T_s/(n_b-n_b_u):
        for n, joint_id in enumerate(revolute_joints):
            # Activation function
            s = ((1 / (1 + math.exp(-m*(n-(n_s-0.5))))) +  1 / (1 + math.exp(-m*((n_s+0.5)-n)))  - 1)

            # Joint Angle calculation
            if n % 4 == 0:
                angle = A_p * math.sin(f_s_p * n + f_t * t) * s + A_lat * math.sin(f_s * n + f_t * t) * (1-s)
            elif n % 4 == 1:
                angle = A_p * math.sin(f_s_p * n + f_t * t + delta)* s + A_dor * math.sin(f_s * n + f_t * t + delta) * (1-s)
            elif n % 4 == 2:
                angle = -A_p * math.sin(f_s_p * n + f_t * t)* s - A_lat * math.sin(f_s * n + f_t * t) * (1-s)
            elif n % 4 == 3:
                angle = -A_p * math.sin(f_s_p * n + f_t * t + delta)* s - A_dor * math.sin(f_s * n + f_t * t + delta) * (1-s)
            p.setJointMotorControl2(
                bodyIndex=robot,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle,
                force=force
            )

            if n_s > 15:
                n_s = 0
            else:
                n_s = n_s + v_ns * d_t

        p.stepSimulation()
        time.sleep(d_t)
        t = time.time()
    
    print("Exist Spiraling")
    
while n_t <= 16:
    t = time.time() - start_time
    L = k_low + (1-k_low)/(1+math.exp(-m_w*(n_t-n_b)))

    for n, joint_id in enumerate(revolute_joints):
        if n <= n_t:
            if n % 4 == 0:
                angle = A_lat * math.sin(f_s * n + f_t * t)
            elif n % 4 == 1:
                angle = A_dor * math.sin(f_s * n + f_t * t + delta)
            elif n % 4 == 2:
                angle = - A_lat * math.sin(f_s * n + f_t * t)
            elif n % 4 == 3:
                angle = - A_dor * math.sin(f_s * n + f_t * t + delta)
            
            # if 0<= n <= n_b - 1:
            #     angle = angle * L
        
        else:
            if n % 4 == 0:
                angle = A_lat * math.sin(f_s * n + f_t * t + d_w)
            elif n % 4 == 1:
                angle = A_dor * math.sin(f_s * n + f_t * t + delta + d_w)
            elif n % 4 == 2:
                angle = - A_lat * math.sin(f_s * n + f_t * t + d_w)
            elif n % 4 == 3:
                angle = - A_dor * math.sin(f_s * n + f_t * t + delta + d_w)
        # Joint Angle calculation

        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=force
        )

    n_t = n_t + v_n_t * d_t


    p.stepSimulation()
    time.sleep(d_t)

print("Exist T-Transition")

while True:
    t = time.time() - start_time
    for n, joint_id in enumerate(revolute_joints):
        if n % 4 == 0:
            angle = beta_lat + A_lat * math.sin(f_s * n + f_t * t)
        elif n % 4 == 1:
            angle = beta_dor + A_dor * math.sin(f_s * n + f_t * t + delta)
        elif n % 4 == 2:
            angle = -beta_lat - A_lat * math.sin(f_s * n + f_t * t)
        elif n % 4 == 3:
            angle = beta_dor - A_dor * math.sin(f_s * n + f_t * t + delta)
        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=force
        )
    p.stepSimulation()
    time.sleep(1. / 240)