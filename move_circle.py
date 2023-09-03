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
        self.DOF = 6 # 6 joints, but 1 is fixed
        self.eef_index = 5
        # reset the robot to the initial position
        self.reset_joints()

        self.force_limits =np.array([100, 100, 100, 100, 100, 100])*0.5

    def reset_joints(self):
        """ reset the robot to the initial position """
        self.move_joints_position(self.init_joint_config)
    # return joint information for all joint states (including fixed joints)
    def getJointStates(self):
        joint_states = p.getJointStates(self.b_id, range(self.DOF))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    # return joint information for the motorized joints only
    # def getMotorJointStates(robot):
    #     joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    #     joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    #     joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    #     joint_positions = [state[0] for state in joint_states]
    #     joint_velocities = [state[1] for state in joint_states]
    #     joint_torques = [state[3] for state in joint_states]
    #     return joint_positions, joint_velocities, joint_torques

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
        zero_vec = [5.0] * len(pos) # base of robot is fixed
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
            targetVelocity=joint_vels,
            forces=[100, 100, 100, 100, 100, 100],
        )
    

def circle3D(t, p, returnVel=True):
    """
    Given t, and starting position p, returns a 3D position (x,y,z) and  that lies on a circle in cartesian space 
    """
    # TODO: shift the circle by the radius so that p is on the circle!
    p = np.array(p) # intitial end effector
    r = .1 # radius of circle
    v1 = np.array([1,0,0])
    v2 = np.array([0,1,0])
    
    pos = p - np.array([r,0,0]) + r*np.cos(t)*v1 + r*np.sin(t)*v2
    if returnVel:
        zero_shift = -np.pi/2.0
        vel = -r*np.sin(t+zero_shift)*v1 + r*np.cos(t+zero_shift)*v2 - np.array([r,0,0]) 
        return  pos, vel
    return pos

if __name__ == "__main__":

    client_id = p.connect(p.GUI)
    # disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # TODO: set the camera position
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
    # info = p.getJointStates(robot.b_id, range(p.getNumJoints(robot.b_id)))
    # print(info)
    # print(len(info))
    # move in a circle
    # get the initial eef position
    init_eef_pos = robot.get_eef_pos() 
    
    # constants
    WAYPOINT_DURATION = 3

    D_GAIN = 0.6
    P_GAIN = 1.0
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
            p.addUserDebugLine(
                eef_desired_vel,
                prev_eef_desired_vel,
                lineColorRGB=[0, 0, 1],
                lineWidth=2.0
                )
        prev_eef_desired_pos  = eef_desired_pos
        prev_eef_desired_vel = eef_desired_vel

        eef_pos = robot.get_eef_pos()
        if t>0: 
            p.addUserDebugLine(
                eef_pos, 
                prev_eef_pos, 
                lineColorRGB=[1, 0.16, 0.02], 
                lineWidth=2.0
                )
        prev_eef_pos  = eef_pos 

        # get the joint command from PD controller
        # TODO: calculate the joint command from PD controller
        # TODO: use the joint command to move the robot
        # TODO: use the joint command to calculate the jacobian matrix
        # TODO: use the jacobian matrix to calculate the end effector velocity

        jac_matrix, link_vr, link_trn = robot.jacobian()
        # compute psuedo-inverse Jacobian
        pinv_jac = np.linalg.pinv(jac_matrix)

        # compute control 
        PD_error = D_GAIN*(eef_desired_vel-link_vr) + P_GAIN*(eef_desired_pos-link_trn)
        # orn = p.getQuaternionFromEuler([np.pi,0., 0.])

        # q_dot = np.matmul(pinv_jac, PD_error)
        q_dot = 0.5*np.sin(6*[v])+0.5*np.cos(6*[v])
        # limit velocities
        # q_dot = np.clip(q_dot, -1.0, 1.0)
        joint_pos, joint_vel, _ =  robot.getJointStates()
        states.append(np.concatenate([joint_pos, joint_vel]))
        actions.append(q_dot)
        # move the robot
        # robot.move_joints_velocity(q_dot)
        # for i in range(24):
        #     print("desired eef velocity:", eef_desired_vel)
        #     print("actual eef velocity:", link_vr)
        #     print(q_dot)
        for i in range(robot.DOF): # the first 7 joints correspond to the arm joints
            p.setJointMotorControl2(robot.b_id, i, 
                                p.VELOCITY_CONTROL, targetVelocity=q_dot[i], 
                                force=robot.force_limits[i]) 

        p.stepSimulation()
        time.sleep(1./240.)
        # t+=0.005
        # if t>10*np.pi:
        #     break
    states = np.array(states)
    actions = np.array(actions)
    np.save("states.npy", states)
    np.save("actions.npy", actions)
    # print(states.shape)