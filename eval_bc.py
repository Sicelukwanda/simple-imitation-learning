import time
import logging
import torch
import pybullet as p
import pybullet_data as pd
from imitation_learning.models import BCPolicy
from imitation_learning.envs import PybulletRobot

import numpy as np

# Helper function to preprocess state (implement as needed)
def preprocess_state(robot_state):
    """Convert the robot's state to the format expected by the BC model."""
    # Example: Convert the robot state to a torch tensor
    return torch.tensor(robot_state, dtype=torch.float32).unsqueeze(0)  # Add batch dimension

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    client = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0')

    p.setGravity(0, 0, -9.8)

    # Disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Load ground plane
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    # Initialize the robot
    cobot = PybulletRobot(client)

    # Load the trained BCPolicy model
    model_path = "trained_bc_policy.pth"

    state_dim = 9
    action_dim = 6
    bc_policy = BCPolicy(state_dim, action_dim, hidden_dims=[256, 256, 256])

    bc_policy.load_state_dict(torch.load(model_path))
    bc_policy.eval()

    # Loop to control the robot
    frequency = 0.1  # Frequency for the control loop (e.g., 10 Hz)
    time_step = 1 / 240.0  # PyBullet simulation timestep

    logging.info("Starting the robot control loop...")

    cobot.reset_joints()

    try:
        while True:
            # Get the current state of the robot
            current_ee_pos = cobot.forward_kinematics()
            current_joint_pos, _, _ = cobot.get_joint_states()
            robot_state = np.concatenate([current_joint_pos, current_ee_pos]) 

            # Preprocess the state for the policy model
            input_state = preprocess_state(robot_state)

            # Predict joint velocities using the policy model
            with torch.no_grad():
                predicted_vels = bc_policy(input_state).squeeze(0).numpy()

            for i in range(int(frequency/time_step)):
                # Command the robot to move based on the predicted velocities
                cobot.move_joints_velocity(predicted_vels) 

                # Step simulation
                p.stepSimulation()

            # Wait for the next control step
            # time.sleep(time_step)

    except KeyboardInterrupt:
        logging.info("Shutting down the robot control loop...")

    finally:
        p.disconnect()