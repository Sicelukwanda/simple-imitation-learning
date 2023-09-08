import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import os

def forward_kinematics(b_id, joint_config):
    """ calculate the end effector position from the joint configuration """
    end_effector_position = p.getLinkState(b_id, 5)[0]
    return end_effector_position

def move_cobot_joints(b_id, joint_config):
    """ use pybullet to move the joints of the mycobot """
    p.setJointMotorControlArray(
        bodyIndex=b_id,
        jointIndices=[0, 1, 2, 3, 4, 5],
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_config,
        forces=[100, 100, 100, 100, 100, 100],
    )


def move_end_effector_in_circle(b_id, radius, frequency):
    """ move the robot end effector in a circle """
    joint_config = np.zeros(6)
    joint_config[0] = radius * np.cos(2 * np.pi * frequency * time.time())
    joint_config[1] = radius * np.sin(2 * np.pi * frequency * time.time())
    move_cobot_joints(b_id, joint_config)

    end_effector_position = forward_kinematics(b_id, joint_config)
    p.addUserDebugLine(end_effector_position, end_effector_position + np.array([0, 0, 0.02]), [1, 0, 0],lineWidth=2.0)


if __name__ == "__main__":

    root_path = "mycobot_description/urdf/mycobot/"

    client = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0')

    p.setGravity(0, 0, -9.8)
    b_id = p.loadURDF(root_path + "mycobot_urdf.urdf", useFixedBase=True)

    # disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # load ground plane
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    frequency = 0.1
    while True:
        move_end_effector_in_circle(b_id, 1.0, frequency)
        p.stepSimulation()
        time.sleep(1./240.)
