import pybullet as p

def visualize_ee(start_vec, end_vec, rgb_color, sim_id, width=2.0):
    """Draw a line segment between two points in the simulation."""
    p.addUserDebugLine(
        start_vec,
        end_vec,
        lineColorRGB=rgb_color,
        lineWidth=width,
        physicsClientId=sim_id,
    )
