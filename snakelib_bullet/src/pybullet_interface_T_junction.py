#!/usr/bin/env python3

import rospy
import numpy as np
import pybullet as p
import pybullet_data
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math
from sympy import symbols, solve, Eq, sin as sym_sin

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

class PyBulletRosWrapper:
    def __init__(self):

        # load params from param server
        self.loop_rate = rospy.get_param("~loop_rate", 100.0)
        self.urdf_path = rospy.get_param("~urdf_path", None)
        self.urdf_spawn_pose = rospy.get_param(
            "~urdf_spawn_pose", [0.0, 0.0, 0.0]
        )  # x,y,z

        self.terrain_path = rospy.get_param("~terrain_path", None)
        self.robot_description = rospy.get_param("~robot_description", None)
        self.tracking_cam = rospy.get_param("~tracking_cam", False)
        self.cam_angle = rospy.get_param(
            "~cam_angle", [2.0, 0, -45]
        )  # dist, yaw, pitch

        self.max_torque = rospy.get_param("~max_torque", 7)  # Nm
        self.joint_control_mode = rospy.get_param(
            "~joint_control_mode", 0
        )  # 0: position, 1: velocity, 3: torque control

        self.joint_control_mode_mapping = {
            0: p.POSITION_CONTROL,
            1: p.VELOCITY_CONTROL,
            2: p.TORQUE_CONTROL,
        }

        self.save_movie = rospy.get_param("~save_movie", 0)  # whether to save a movie

        # start services
        rospy.Service("~reset_simulation", Empty, self.handle_reset_simulation)
        rospy.Service("~pause_physics", Empty, self.handle_pause_physics)
        rospy.Service("~unpause_physics", Empty, self.handle_unpause_physics)

        # start pybullet
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setTimeStep(1.0 / self.loop_rate)

        self.robot_id = 0
        self.num_dofs = 0
        self.pause_simulation = False

        # these need to be defined everytime we reset the sim
        p.setGravity(0, 0, -9.81)
        self.load_terrain()
        self.load_t_junction()
        self.load_urdf()

        # placeholder for storing joint command data
        self.command_positions = [0.0] * self.num_dofs
        self.command_velocities = [0.0] * self.num_dofs

        # initialize effort as max torque for position, velocity control:
        # setting it to zero causes the robot to move when not giving any commands
        if self.joint_control_mode != 2:
            self.command_efforts = [self.max_torque] * self.num_dofs
        else:
            self.command_efforts = [0.0] * self.num_dofs

        # TODO: parse max torques from URDF

        # start joint command subscriber
        self.joint_command_topic = "/snake/joint_commands"
        rospy.Subscriber(
            self.joint_command_topic, JointState, self.joint_command_callback
        )

        # initialize publishers
        self.joint_state_pub = rospy.Publisher(
            "/snake/joint_states", JointState, queue_size=100
        )
        
        self.imu_pub = rospy.Publisher(
            "/imu/data_raw", Imu, queue_size=10
        )
        self.prev_time = rospy.Time.now().to_sec()
        self.prev_velocity = [0, 0, 0]
        self.prev_orientation = [0, 0, 0, 1]

        # TODO: implement a way to simulate IMU readings OR publish orientations directly
        # imu_pub = rospy.Publisher('/snake/joint_states', , queue_size=100)

        self.joint_state = JointState()

    def start(self):

        self.joint_state.name = self.revolute_joint_idx.values()

        rate = rospy.Rate(self.loop_rate)

        if self.save_movie:
            self.start_video_log()

        while not rospy.is_shutdown():

            self.send_joint_commands()

            time_now = rospy.Time.now()
            self.joint_state.header.stamp = time_now
            (
                self.joint_state.position,
                self.joint_state.velocity,
                self.joint_state.effort,
            ) = self.get_feedback()

            self.joint_state_pub.publish(self.joint_state)

            # TODO: IMU reading publisher
            if self.tracking_cam:
                self.update_cam_view(self.cam_angle)

            if not self.pause_simulation:
                p.stepSimulation()
            rate.sleep()

        # Save movie if necessary
        if self.save_movie:
            self.stop_video_log()

    def enable_joint_torque_sensor(self):
        for i in self.joint_idx:
            p.enableJointForceTorqueSensor(self.robot_id, i, True)

    def get_feedback(self):
        # Get joint state (position, velocity and output torques).

        
        position = np.array([0.0] * self.num_dofs)
        velocity = np.array([0.0] * self.num_dofs)
        effort = np.array([0.0] * self.num_dofs)
        
        imu_data = self.compute_imu(1,-1)
        
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = imu_data['linear_acceleration'][0]
        imu_msg.linear_acceleration.y = imu_data['linear_acceleration'][1]
        imu_msg.linear_acceleration.z = imu_data['linear_acceleration'][2]

        imu_msg.angular_velocity.x = imu_data['angular_velocity'][0]
        imu_msg.angular_velocity.y = imu_data['angular_velocity'][1]
        imu_msg.angular_velocity.z = imu_data['angular_velocity'][2]

        imu_msg.orientation.x = imu_data['orientation'][0]
        imu_msg.orientation.y = imu_data['orientation'][1]
        imu_msg.orientation.z = imu_data['orientation'][2]
        imu_msg.orientation.w = imu_data['orientation'][3]
        
        roll, pitch, yaw = euler_from_quaternion([imu_data['orientation'][0],imu_data['orientation'][1],imu_data['orientation'][2],imu_data['orientation'][3]])
        # print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        self.imu_pub.publish(imu_msg)

        for idx, motor_no in enumerate(list(self.joint_idx)):
            joint_p, joint_v, _, output_torque = p.getJointState(
                self.robot_id, motor_no
            )
            position[idx] = joint_p
            velocity[idx] = joint_v
            effort[idx] = output_torque

        return (
            position.tolist(),
            velocity.tolist(),
            effort.tolist(),
        )

    def send_joint_commands(self):
        p.setJointMotorControlArray(
            self.robot_id,
            self.joint_idx,
            self.joint_control_mode_mapping[self.joint_control_mode],
            targetPositions=self.command_positions,
            targetVelocities=self.command_velocities,
            forces=self.command_efforts,
        )

    def joint_command_callback(self, msg):

        # This method parses the joint commands based on the control mode specified by the ROS param joint_control_mode.
        # Refer pybullet's setJointMotorControlArray method definition for more information about this implementation.

        """There is currently a discrepency between URDF joint names and the module names.
        This causes PyBullet to crash when we provide the module names. We do not want to
        require the URDF to match the module names, because the modules often get replaced
        and this would make this cumbersome. Instead, we should write a node that converts
        module names into URDF names and provides this to PyBullet. For now, we remove the
        names to make integration possible until we can implement this node.
        """
        msg.name = []

        if self.joint_control_mode == 0:
            # POSITION_CONTROL

            # make a dictionary of joint_name: command and send it to sort_joint_commands() for sorting
            self.command_positions = self.sort_joint_commands(msg.name, msg.position)

            # send sorted velocity commands if received, else send defaults.
            self.command_velocities = (
                self.sort_joint_commands(msg.name, msg.velocity)
                if len(msg.velocity) == self.num_dofs
                else [0.0] * self.num_dofs
            )

            # We may fill the joint state with NaNs for the HEBI actuators, but PyBullet cannot take NaNs
            self.command_velocities = np.nan_to_num(self.command_velocities)

            # send sorted effort commands (max torques) if received, else send defaults.
            # In position contorl mode, forces are the maximum motor force used to reach target position
            self.command_efforts = (
                self.sort_joint_commands(msg.name, msg.effort)
                if len(msg.effort) == self.num_dofs
                else [self.max_torque] * self.num_dofs
            )

        elif self.joint_control_mode == 1:
            # VELOCITY_CONTROL

            # pybullet implementation of velocity control does not use position commands, so we set them to defaults.
            self.command_positions = [0.0] * self.num_dofs

            # send sorted velocity commands
            self.command_velocities = self.sort_joint_commands(msg.name, msg.velocity)

            # send sorted effort commands (max torques) if received, else send defaults.
            self.command_efforts = (
                self.sort_joint_commands(msg.name, msg.effort)
                if len(msg.effort) == self.num_dofs
                else [self.max_torque] * self.num_dofs
            )

        elif self.joint_control_mode == 2:
            # TORQUE_CONTROL

            # pybullet implementation of torque control does not use position and velocity commands, so we set them to defaults.
            self.command_positions = [0.0] * self.num_dofs
            self.command_velocities = [0.0] * self.num_dofs

            # send sorted effort commands.
            self.command_efforts = self.sort_joint_commands(msg.name, msg.effort)

    def sort_joint_commands(self, name, command):

        # If joint names list is not specified/incomplete, warn user and return unsorted commands.
        if len(name) < self.num_dofs:
            rospy.logwarn_once(
                f"Message from topic {self.joint_command_topic} didn't contain a joint name list. Using unsorted commands.",
            )
            return command
        else:
            joint_commands_with_names = dict(zip(name, command))
            commands = []
            for joint_idx in sorted(self.revolute_joint_idx):
                commands.append(
                    joint_commands_with_names.get(
                        self.revolute_joint_idx.get(joint_idx)
                    )
                )
        return commands

    def update_cam_view(self):
        pos_sum = np.zeros(3)
        distance, pitch, yaw = self.cam_angle
        pos, *_ = p.getLinkState(self.robot_id, self.joint_idx[int(self.num_dofs / 2)])
        p.resetDebugVisualizerCamera(distance, pitch, yaw, pos)
        return 0

    def load_urdf(self):
        self.robot_id = p.loadURDF(
            self.urdf_path,
            self.urdf_spawn_pose,
            useFixedBase=0,
            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )
        (
            self.revolute_joint_idx,
            self.prismatic_joint_idx,
            self.fixed_joint_idx,
            self.link_name_idx,
        ) = self.get_properties()
        self.num_dofs = len(self.revolute_joint_idx)
        self.joint_idx = list(self.revolute_joint_idx.keys())
    
    def load_t_junction(self):
        # ---------- T-junction pipe (two separate static cylinders) ----------
        # orientations: cylinder axis is local Z
        # main pipe along X: rotate Z -> X by +90° about Y
        orn_main = p.getQuaternionFromEuler([0, 0, 0])
        # branch along Y: rotate Z -> Y by -90° about X
        orn_branch = p.getQuaternionFromEuler([-math.pi / 2, 0, 0])

        # T-junction parameters
        main_position = [0.1, 0.5, 0.5]
        branch_position = [0.1, 0.5, 1]
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


    def get_properties(self):
        # method to parse robot model and construct 4 dictionaries:
        # revolute_joint_idx: map revolute joint indices to joint names (in URDF)
        # fixed_joint_idx: map fixed joint indices to joint names (in URDF)
        # prismatic_joint_idx: map prismatic joint indices to joint names (in URDF)
        # link_name_idx: map link names (in URDF) to joint indices
        revolute_joint_idx = {}  # (joint index: joint name)
        fixed_joint_idx = {}  # (joint index: joint name)
        prismatic_joint_idx = {}  # (joint index: joint name)
        link_name_idx = {}  # (link name: joint index)
        for idx in range(0, p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, idx)
            link_name_idx[info[12].decode("utf-8")] = idx
            if info[2] == p.JOINT_REVOLUTE:
                revolute_joint_idx[idx] = info[1].decode("utf-8")
            elif info[2] == p.JOINT_FIXED:
                fixed_joint_idx[idx] = info[1].decode("utf-8")
            elif info[2] == p.JOINT_PRISMATIC:
                prismatic_joint_idx[idx] = info[1].decode("utf-8")
            else:  # Continuous joints are not supported by getJointInfo, so otherwise assume its a revolute joint
                revolute_joint_idx[idx] = info[1].decode("utf-8")
        return revolute_joint_idx, prismatic_joint_idx, fixed_joint_idx, link_name_idx

    def load_terrain(self):
        p.loadURDF("plane.urdf")
        try:
            if self.terrain_path is not None:
                p.loadURDF(self.terrain_path, useFixedBase=1)
        except p.error as e:
            print(
                f'Failed to load terrain URDF. Does the file exist? PyBullet error message: "{e}"'
            )

    def handle_reset_simulation(self, req):
        rospy.loginfo("reseting simulation now")
        self.pause_simulation = True
        p.resetSimulation()
        # load URDF model again, set gravity and floor
        p.setGravity(0, 0, -9.81)
        self.load_terrain()
        self.robot_id = self.load_urdf()
        self.pause_simulation = False
        return []

    def handle_pause_physics(self, req):
        rospy.loginfo("pausing simulation")
        self.pause_simulation = True
        return []

    def handle_unpause_physics(self, req):
        rospy.loginfo("unpausing simulation")
        self.pause_simulation = False
        return []

    """
    Starts the logging to record a video of the bullet simulation
    """

    def start_video_log(self, filename="/data/ros/pb_video.mp4"):
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, filename)
        return 0

    """
    Stops the logging of the bullet simulation video
    """

    def stop_video_log(self, filename="/data/ros/pb_video.mp4"):
        p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4, filename)
        return 0

    def update_cam_view(self, dist=4.5, yaw=45, pitch=-45):

        # Get the position of the center module
        pos, *_ = p.getLinkState(
            self.robot_id, self.revolute_joint_idx[int(self.num_dofs / 2) - 1]
        )

        _pos = list(pos)
        _pos[2] = 0
        pos = tuple(_pos)

        p.resetDebugVisualizerCamera(dist, yaw, pitch, pos)
        return 0
    
    def compute_imu(self,robot_id, link_index):
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.prev_time
        if dt == 0:
            return None  # avoid division by zero

        # Get base link velocity and orientation
        linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)

        # Linear acceleration (finite difference)
        linear_acc = [(v2 - v1)/dt for v1, v2 in zip(self.prev_velocity, linear_velocity)]

        # Store for next timestep
        self.prev_velocity = linear_velocity
        self.prev_time = current_time
        self.prev_orientation = orientation

        return {
            'linear_acceleration': linear_acc,
            'angular_velocity': angular_velocity,
            'orientation': orientation
        }


if __name__ == "__main__":
    rospy.init_node("pybullet_interface", anonymous=False)
    pybullet_interface = PyBulletRosWrapper()
    try:
        pybullet_interface.start()
    except rospy.ROSInterruptException:
        pass
