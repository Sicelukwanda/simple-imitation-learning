import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import os

class PybulletRobot():
    def __init__(self, urdf_path, sim_client_id):
        self.sim_client_id = sim_client_id
        self.b_id = p.loadURDF(urdf_path, useFixedBase=True, physicsClientId=self.sim_client_id)
        self.init_joint_config = np.zeros(6, dtype=np.float32)
        self.init_joint_config[2] = -np.pi/2
        self.init_joint_config[5] = -np.pi/2
        self.DOF = 6  # 6 joints, but 1 is fixed
        self.eef_index = 5
        self.reset_joints()
        self.force_limits = np.array([100, 100, 100, 100, 100, 100]) * 0.5

    def reset_joints(self):
        """Reset the robot to the initial position."""
        self.move_joints_position(self.init_joint_config)
        for _ in range(20):
            p.stepSimulation()

    def getJointStates(self):
        """Return joint information for all joint states (including fixed joints)."""
        joint_states = p.getJointStates(self.b_id, range(self.DOF))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def jacobian(self):
        """Calculate the Jacobian matrix of mycobot (and velocity of eef link)."""
        result = p.getLinkState(self.b_id,
                               self.eef_index,
                               computeLinkVelocity=1,
                               computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

        pos, vel, torq = robot.getJointStates()
        zero_vec = [0.0] * len(pos)  # base of robot is fixed
        jac_t, jac_r = p.calculateJacobian(self.b_id, self.eef_index, com_trn, pos, zero_vec, zero_vec)

        return np.array(jac_t, dtype=np.float32), np.array(link_vr, dtype=np.float32), np.array(link_trn, dtype=np.float32)

    def get_eef_pos(self):
        """Calculate the end effector position from the joint configuration."""
        end_effector_position = p.getLinkState(self.b_id, 5)[0]
        return np.array(end_effector_position, dtype=np.float32)

    def move_joints_position(self, joint_config):
        """Use pybullet to move the joints of the mycobot with VELOCITY control."""
        p.setJointMotorControlArray(
            bodyIndex=self.b_id,
            jointIndices=[0, 1, 2, 3, 4, 5],
            controlMode=p.POSITION_CONTROL,
            targetPositions=joint_config,
            forces=[100, 100, 100, 100, 100, 100],
        )

    def move_joints_velocity(self, joint_vels):
        """Use pybullet to move the joints of the mycobot with VELOCITY control."""
        p.setJointMotorControlArray(
            bodyIndex=self.b_id,
            jointIndices=[0, 1, 2, 3, 4, 5],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=joint_vels,
            forces=[100, 100, 100, 100, 100, 100],
        )

def circle_path(t, p, returnVel=True):
    """
    Given t (time in seconds) and starting position p, returns a 3D position (x, y, z) and velocity
    that lies on a circle in cartesian space.
    """
    p = np.array(p)  # Initial end effector
    r = 0.05  # Radius of circle
    v1 = np.array([0, 1, 0])
    v2 = np.array([0, 0, 1])

    pos = p - np.array([0, r, 0]) + r * np.cos(t) * v1 + r * np.sin(t) * v2
    if returnVel:
        zero_shift = -np.pi/2.0
        vel = -r * np.sin(t + zero_shift) * v1 + r * np.cos(t + zero_shift) * v2 - np.array([r, 0, 0])
        return pos, vel
    return pos

def visualize(start_vec, end_vec, rgb_color, sim_id):
    """Draw a line segment in x, y, z between start_vec and end_vec."""
    p.addUserDebugLine(
        start_vec,
        end_vec,
        lineColorRGB=rgb_color,
        lineWidth=2.0,
        physicsClientId=sim_id
    )

if __name__ == "__main__":
    # Start a simulator client
    client_id = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0')


    # Disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Set the viewing position
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=135,
        cameraPitch=-50,
        cameraTargetPosition=[0.5, 0.5, 0.5],
        physicsClientId=client_id,
    )

    # Load ground plane
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    PHYSICS_TIME_STEP = 1.0 / 240.0
    p.setTimeStep(PHYSICS_TIME_STEP)

    urdf_path = "mycobot_description/urdf/mycobot/mycobot_urdf.urdf"
    robot = PybulletRobot(urdf_path, client_id)

    # Get the initial eef position
    init_eef_pos = robot.get_eef_pos()

    # Hand-tuned gains
    D_GAIN = 0.2
    P_GAIN = 5.0
    SIM_DURATION = 10  # Time in seconds
    TOTAL_SIM_STEPS = int(SIM_DURATION / PHYSICS_TIME_STEP)  # Seconds

    traj_times = np.linspace(0.0, 2 * np.pi , TOTAL_SIM_STEPS)
    alpha = 1.5
    frequency = 0.1
    t = 0
    prev_eef_pos = None
    prev_eef_desired_pos = None
    prev_eef_desired_vel = None

    states = []
    actions = []

    for t in range(TOTAL_SIM_STEPS):
        v = traj_times[t] * 1.5
        eef_desired_pos, eef_desired_vel = circle_path(v, init_eef_pos, returnVel=True)
        if t > 0:
            p.addUserDebugLine(
                eef_desired_pos,
                prev_eef_desired_pos,
                lineColorRGB=[0, 1, 0.02],
                lineWidth=2.0
            )

        prev_eef_desired_pos = eef_desired_pos

        eef_pos = robot.get_eef_pos()

        if t > 0:
            visualize(
                eef_pos,
                prev_eef_pos,
                [1, 0.16, 0.02],
                client_id
            )
        prev_eef_pos = eef_pos

        pos_jac_matrix, link_vr, link_trn = robot.jacobian()

        pinv_jac = np.linalg.pinv(pos_jac_matrix)

        eef_vel = prev_eef_pos - eef_pos
        PD_error = D_GAIN * (eef_desired_vel - eef_vel) + P_GAIN * (eef_desired_pos - eef_pos)

        q_dot = np.matmul(pinv_jac, PD_error)

        joint_pos, joint_vel, _ = robot.getJointStates()
        states.append(np.concatenate([joint_pos, joint_vel]))
        actions.append(q_dot)

        robot.move_joints_velocity(q_dot)

        p.stepSimulation()

    states = np.array(states)
    actions = np.array(actions)
    np.save("states.npy", states)
    np.save("actions.npy", actions)
