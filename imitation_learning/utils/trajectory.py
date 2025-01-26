import numpy as np

def circle_path(t, p, radius=0.05, return_vel=True):
    """
    Generate a circular path in Cartesian space.

    Parameters:
        t (float): Time parameter.
        p (list): Initial position of the end effector.
        radius (float): Radius of the circle.
        return_vel (bool): Whether to return velocity.

    Returns:
        pos (np.ndarray): Desired position.
        vel (np.ndarray): Desired velocity (if return_vel=True).
    """
    p = np.array(p)
    v1 = np.array([0, 1, 0])
    v2 = np.array([0, 0, 1])

    pos = p - np.array([0, radius, 0]) + radius * np.cos(t) * v1 + radius * np.sin(t) * v2
    if return_vel:
        vel = -radius * np.sin(t) * v1 + radius * np.cos(t) * v2
        return pos, vel
    return pos
