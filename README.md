# Robot Learning Workshop Tutorial @ Deep Learning Indaba 2023
Website: [https://sites.google.com/view/robotlearning4africa/home](https://sites.google.com/view/robotlearning4africa/home)

![](data/cobot.png)
## Setup (tested with Python 3.8)
Within a Python environment install `numpy` and `pybullet`, i.e., `pip install pybullet`

Use this repo to collect data for training a Behavioural Cloning (BC) model. To help you along we have included python scripts:


1. Environment and Robot Setup
Purpose: Handles PyBullet robot setup, and simulation logic.
Target File: `envs/pybullet_env.py`
2. Control Logic
Purpose: Encapsulates the PD controller for controlling the robot's motion. This controller issues commands using a [Proportional Derivative (PD) Controlller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
Target File: `controllers/pd_controller.py`
3. Trajectory Generation
Purpose: Handles the generation of desired trajectories for the robot.
Target File: `utils/trajectory.py`
4. Visualization
Purpose: Provides functions for trajectory visualization and debug drawing.
Target File: `utils/visualizers.py`