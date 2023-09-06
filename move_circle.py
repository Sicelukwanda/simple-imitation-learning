import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import os

class PybulletRobot():
    def __init__(self, urdf_path, sim_client_id):
        self.sim_client_id = sim_client_id
        self.b_id = p.loadURDF(
            urdf_path, 
            useFixedBase=True, 
            physicsClientId=self.sim_client_id
            )
        self.init_joint_config = np.zeros(6, dtype=np.float32)
        self.init_joint_config[2] = -np.pi/2
        self.init_joint_config[5] = -np.pi/2

        self.DOF = 6 # 6 joints, but 1 is fixed
        self.eef_index = 5
        # reset the robot to the initial position
        self.reset_joints()

        self.force_limits =np.array([100, 100, 100, 100, 100, 100])*0.5

    def reset_joints(self):
        """ reset the robot to the initial position """
        self.move_joints_position(self.init_joint_config)
        for i in range(20):
            p.stepSimulation()
    # return joint information for all joint states (including fixed joints)
    def getJointStates(self):
        joint_states = p.getJointStates(self.b_id, range(self.DOF))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques


    def forward_kinematics(self):
        """ calculate the end effector position from the joint configuration """
        pass

    def jacobian(self):
        """ calculate the jacobian matrix of mycobot (and velocity of eef link)"""
        result = p.getLinkState(self.b_id,
                        self.eef_index,
                        computeLinkVelocity=1,
                        computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

        # Get the Jacobians for the CoM of the end-effector link.
        # Note that in this example com_rot = identity, and we would need to use com_rot.T * com_trn.
        # The localPosition is always defined in terms of the link frame coordinates.
        pos, vel, torq = robot.getJointStates()
        zero_vec = [0.0] * len(pos) # base of robot is fixed
        # compute translation and rotation Jacobian
        jac_t, jac_r = p.calculateJacobian(self.b_id, self.eef_index, com_trn, pos, zero_vec, zero_vec)

        return np.array(jac_t, dtype=np.float32), np.array(link_vr, dtype=np.float32), np.array(link_trn, dtype=np.float32)

    def get_eef_pos(self):
        """ calculate the end effector position from the joint configuration """
        end_effector_position = p.getLinkState(self.b_id, 5)[0]
        return np.array(end_effector_position, dtype=np.float32)

    def move_joints_position(self, joint_config):
        """ use pybullet to move the joints of the mycobot with VELOCITY control"""
        p.setJointMotorControlArray(
            bodyIndex=self.b_id,
            jointIndices=[0, 1, 2, 3, 4, 5],
            controlMode=p.POSITION_CONTROL,
            targetPositions=joint_config,
            forces=[100, 100, 100, 100, 100, 100],
        )

    def move_joints_velocity(self, joint_vels):
        """ use pybullet to move the joints of the mycobot with VELOCITY control"""
        p.setJointMotorControlArray(
            bodyIndex=self.b_id,
            jointIndices=[0, 1, 2, 3, 4, 5],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=joint_vels,
            forces=[100, 100, 100, 100, 100, 100],
        )
    

def circle3D(t, p, returnVel=True):
    """
    Given t (time in seconds), and starting position p, returns a 3D position (x,y,z) and  that lies on a circle in cartesian space 
    """
    p = np.array(p) # intitial end effector
    r = .05 # radius of circle
    v1 = np.array([0,1,0])
    v2 = np.array([0,0,1])
    
    pos = p - np.array([0,r,0]) + r*np.cos(t)*v1 + r*np.sin(t)*v2
    if returnVel:
        zero_shift = -np.pi/2.0
        vel = -r*np.sin(t+zero_shift)*v1 + r*np.cos(t+zero_shift)*v2 - np.array([r,0,0]) 
        return  pos, vel
    return pos

def visualize(start_vec, end_vec, rgb_color, sim_id):
    """Draw line segment in x,y,z between start_vec and end_vec"""
    p.addUserDebugLine(
                start_vec, 
                end_vec, 
                lineColorRGB = rgb_color, 
                lineWidth = 2.0,
                physicsClientId = sim_id
                )

if __name__ == "__main__":

    # start a simulator client
    client_id = p.connect(p.GUI)

    # disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # set the viewing position
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=135,
        cameraPitch=-50,
        cameraTargetPosition=[0.5, 0.5, 0.5],
        physicsClientId=client_id,
    )

    # load ground plane
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    PHYSICS_TIME_STEP = 1.0 / 240.0
    p.setTimeStep(PHYSICS_TIME_STEP)

    urdf_path = "mycobot_description/urdf/mycobot/mycobot_urdf.urdf"
    robot = PybulletRobot(urdf_path,client_id)

    # get the initial eef position
    init_eef_pos = robot.get_eef_pos() 
    
    # handtuned gains
    D_GAIN = 0.2
    P_GAIN = 5.0
    SIM_DURATION = 10 # time in seconds
    TOTAL_SIM_STEPS = int(SIM_DURATION/PHYSICS_TIME_STEP) # seconds

    traj_times = np.linspace(0.0,2*np.pi + np.pi,TOTAL_SIM_STEPS)
    alpha = 1.5
    frequency = 0.1
    t = 0
    prev_eef_pos = None
    prev_eef_desired_pos = None
    prev_eef_desired_vel = None
    
    states = []
    actions = []

    for t in range(TOTAL_SIM_STEPS):
        # move_end_effector_in_circle(b_id, 1.0, frequency)
        v = traj_times[t]*1.5
        eef_desired_pos, eef_desired_vel = circle3D(v, init_eef_pos, returnVel=True)
        if t>0: 
            
            p.addUserDebugLine(
                eef_desired_pos, 
                prev_eef_desired_pos, 
                lineColorRGB=[0, 1, 0.02], 
                lineWidth=2.0
                )

        prev_eef_desired_pos  = eef_desired_pos

        eef_pos = robot.get_eef_pos()

        if t > 0: 
            visualize(
                eef_pos, 
                prev_eef_pos, 
                [1, 0.16, 0.02], 
                client_id
                )
        prev_eef_pos  = eef_pos 

        # compute Jacobian (position only)
        pos_jac_matrix, link_vr, link_trn = robot.jacobian()

        # compute psuedo-inverse Jacobian
        pinv_jac = np.linalg.pinv(pos_jac_matrix)
           
        
        # compute control 
        eef_vel = prev_eef_pos - eef_pos
        PD_error = D_GAIN*(eef_desired_vel - eef_vel) + P_GAIN*(eef_desired_pos - eef_pos)

        # use jacobian to convert end-effector velocities to joint velocities            
        q_dot = np.matmul(pinv_jac, PD_error)

        # save robot joint states
        joint_pos, joint_vel, _ =  robot.getJointStates()
        states.append(np.concatenate([joint_pos, joint_vel]))
        actions.append(q_dot)

        # move the robot
        robot.move_joints_velocity(q_dot)

        p.stepSimulation()


    states = np.array(states)
    actions = np.array(actions)
    np.save("states.npy", states)
    np.save("actions.npy", actions)
