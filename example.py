import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import logging

from imitation_learning.robot_models import get_urdf_path
from imitation_learning.envs import PybulletRobot


def follow_8(robot, radius, frequency):
    """ move the robot end effector in a figure 8 """
    joint_config = np.zeros(6)
    joint_config[0] = radius * np.cos(2 * np.pi * frequency * time.time())
    joint_config[1] = radius * np.sin(2 * np.pi * frequency * time.time())

    logging.info(f"joint_config: {joint_config}")

    robot.move_joints_position(joint_config)

    end_effector_position = robot.forward_kinematics()
    p.addUserDebugLine(end_effector_position, end_effector_position + np.array([0, 0, 0.02]), [1, 0, 0],lineWidth=2.0)


if __name__ == "__main__":

    logging.basicConfig(level=logging.INFO)


    urdf_robot_path = get_urdf_path("mycobot")

    client = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0')

    p.setGravity(0, 0, -9.8)

    # disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # load ground plane
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    # load robot
    cobot = PybulletRobot(client)
    
    frequency = 0.1
    radius = 1.0
    while True:
        
        follow_8(cobot, radius, frequency)
        p.stepSimulation()
        time.sleep(1./240.)
