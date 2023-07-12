import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import os

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


if __name__ == "__main__":

    root_path = "mycobot_description/urdf/mycobot/"

    client = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    b_id = p.loadURDF(root_path + "mycobot_urdf.urdf", useFixedBase=True)

    # disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # load ground plane
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    frequency = 0.1
    while True:
        move_end_effector_in_circle(b_id, 0.5, frequency)
        p.stepSimulation()
        time.sleep(1./240.)
