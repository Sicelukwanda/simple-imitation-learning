# Robot Learning Tutorial @ Deep Learning Indaba 2023

![](data/cobot.png)
## Setup (tested with Python 3.8)
Install `numpy` and `pybullet`, i.e., `pip install pybullet`

Use this repo to collect data for training your Behavioural Cloning (BC) model. To help you along we have included python scripts:

- `move_basic.py` simple script that moves the simulated robot in a figure 8 pattern.
- `move_circle.py` advanced script that moves the simulated robot in a circular path using a [Proportional Derivative (PD) Controlller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
