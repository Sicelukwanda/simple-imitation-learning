import pybullet as p
import pybullet_data as pd
import time
import numpy as np
from imitation_learning.envs import PybulletRobot
from imitation_learning.controllers import PDController
from imitation_learning.utils.trajectory import circle_path
from imitation_learning.utils.visualizers import visualize_ee

import logging

if __name__ == "__main__":
    
    PHYSICS_TIME_STEP = 1.0 / 240.0
    logging.basicConfig(level=logging.INFO)

    client_id = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0')

    p.setGravity(0, 0, -9.8)

    # disable debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # load ground plane
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    robot = PybulletRobot(sim_client_id=client_id)
    pd_controller = PDController(robot, p_gain=5.0, d_gain=0.2)

    init_eef_pos = robot.forward_kinematics()
    total_sim_steps = int(10 / (PHYSICS_TIME_STEP))
    states, actions = [], []

    # Initialize variables for visualization
    prev_eef_pos = None
    prev_desired_pos = None

    # time keeping
    time_elapsed = 0.0

    for t in np.linspace(0, 2 * np.pi, total_sim_steps):
        # Generate desired position and velocity
        desired_pos, desired_vel = circle_path(t, init_eef_pos)

        time_elapsed+=PHYSICS_TIME_STEP

        # Get current position and velocity
        current_pos = robot.forward_kinematics()

        logging.info(f"Seconds:{time_elapsed} Current pos: {current_pos}, Desired pos: {desired_pos}")

        # Compute joint velocities using the PD controller
        vel_control_signal = pd_controller(desired_pos, desired_vel)

        # Apply joint velocities to the robot
        robot.move_joints_velocity(vel_control_signal)

        # Visualization of trajectories
        if prev_eef_pos is not None and prev_desired_pos is not None:
            # Draw the current end-effector trajectory in red
            visualize_ee(prev_eef_pos, current_pos, [1, 0, 0], client_id)
            # Draw the desired trajectory in green
            visualize_ee(prev_desired_pos, desired_pos, [0, 1, 0], client_id)

        # Update the previous positions for the next iteration
        prev_eef_pos = current_pos
        prev_desired_pos = desired_pos

        # Step the simulation
        p.stepSimulation()
        time.sleep(PHYSICS_TIME_STEP)

        # Collect state-action pairs
        states.append(current_pos)
        actions.append(vel_control_signal)

    # Save the data
    np.save("states.npy", np.array(states))
    np.save("actions.npy", np.array(actions))
