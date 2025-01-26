import pybullet as p

def visualize_ee(start_vec, end_vec, rgb_color, sim_id, width=2.0):
    """
    Draw a line segment between two points in the simulation.

    Parameters:
        start_vec (list or tuple): Starting point of the line segment (x, y, z).
        end_vec (list or tuple): Ending point of the line segment (x, y, z).
        rgb_color (list or tuple): RGB color of the line as a list (e.g., [1, 0, 0] for red).
        sim_id (int): The physics client ID of the PyBullet simulation.
        width (float): The width of the line segment (default is 2.0).
    """
    
    if not (len(start_vec) == 3 and len(end_vec) == 3):
        raise ValueError("start_vec and end_vec must each have exactly 3 elements (x, y, z).")
    if not (len(rgb_color) == 3):
        raise ValueError("rgb_color must have exactly 3 elements (r, g, b).")

    p.addUserDebugLine(
        start_vec,
        end_vec,
        lineColorRGB=rgb_color,
        lineWidth=width,
        physicsClientId=sim_id,
    )
