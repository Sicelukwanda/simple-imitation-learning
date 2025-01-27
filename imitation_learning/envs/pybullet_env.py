import pybullet as p
import numpy as np
from dataclasses import dataclass
from imitation_learning.robot_models import get_urdf_path

@dataclass
class RobotConfig:

    urdf_path: str
    DOF: int
    init_joint_config: np.ndarray
    force_limits: np.ndarray
    eef_index: int


class PybulletRobot:

    def __init__(self, sim_client_id: int, robot_config: RobotConfig = None):

        self.sim_client_id = sim_client_id
        # sim reset steps

        # let controller move robot to reset state
        self.SIM_RESET_STEPS = 20 

        if robot_config is None:
            # use mycobot config
            init_joint_config = np.zeros(6, dtype=np.float32)
            init_joint_config[2] = -np.pi / 2
            init_joint_config[5] = -np.pi / 2

            robot_config = RobotConfig(
                urdf_path=get_urdf_path("mycobot"),
                DOF=6,
                init_joint_config=init_joint_config,
                force_limits=np.array([100, 100, 100, 100, 100, 100], dtype=np.float32)* 0.5,
                eef_index=5
            )
           
        self.urdf_path = robot_config.urdf_path
        self.DOF = robot_config.DOF
        self.init_joint_config = robot_config.init_joint_config
        self.force_limits = robot_config.force_limits
        self.eef_index = robot_config.eef_index
        

        self.b_id = p.loadURDF(self.urdf_path, useFixedBase=True, physicsClientId=self.sim_client_id)
        self.reset_joints()
        
    def reset_joints(self):
        """Reset the robot to its initial position."""
        self.move_joints_position(self.init_joint_config)
        for _ in range(self.SIM_RESET_STEPS):
            p.stepSimulation()

    def get_joint_states(self):
        """Return joint positions, velocities, and torques."""
        joint_states = p.getJointStates(self.b_id, range(self.DOF))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return np.array(joint_positions, dtype=np.float32), np.array(joint_velocities, dtype=np.float32), np.array(joint_torques, dtype=np.float32)

    def jacobian(self):
        """Calculate the Jacobian matrix of mycobot (and velocity of eef link)."""
        result = p.getLinkState(self.b_id,
                                self.eef_index,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

        pos, vel, torq = self.get_joint_states()
        zero_vec = [0.0] * len(pos)  # base of robot is fixed
        jac_t, jac_r = p.calculateJacobian(self.b_id, self.eef_index, com_trn, list(pos), zero_vec, zero_vec)

        return np.array(jac_t, dtype=np.float32), np.array(link_vr, dtype=np.float32), np.array(link_trn, dtype=np.float32)

    
    def forward_kinematics(self, b_id=-1):
        """ calculate the end effector position from the joint configuration """
        if b_id == -1:
            end_effector_position = p.getLinkState(self.b_id, self.eef_index)[0]
        else:
            end_effector_position = p.getLinkState(b_id, self.eef_index)[0]
        return np.array(end_effector_position, dtype=np.float32)
    
    def inverse_kinematics(self, target_pos):
        raise NotImplementedError

    def move_joints_position(self, joint_config):
        p.setJointMotorControlArray(
            bodyIndex=self.b_id,
            jointIndices=range(self.DOF),
            controlMode=p.POSITION_CONTROL,
            targetPositions=joint_config,
            forces=self.force_limits,
        )

    def move_joints_velocity(self, joint_vels):
        p.setJointMotorControlArray(
            bodyIndex=self.b_id,
            jointIndices=range(self.DOF),
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=joint_vels,
            forces=self.force_limits,
        )
