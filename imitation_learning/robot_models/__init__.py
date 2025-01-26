import pkg_resources
import os

ROBOT_URDF_REGISTRY = {
    "mycobot": "robot_models/mycobot_description/urdf/mycobot/mycobot_urdf.urdf",
    # Add additional robot entries as needed
    # "robot_name": "path_to_urdf/urdf_file_name"
}

def get_urdf_path(robot_name="mycobot"):
    """
    Retrieve the full path to the URDF file of a specific robot model.

    Parameters:
        robot_name (str): The name of the robot model. Defaults to "mycobot".

    Returns:
        str: The full path to the URDF file.

    Raises:
        ValueError: If the robot name is not registered in the ROBOT_URDF_REGISTRY.
    """
    if robot_name not in ROBOT_URDF_REGISTRY:
        raise ValueError(
            f"Robot name '{robot_name}' not found in the registry. "
            f"Available robots: {list(ROBOT_URDF_REGISTRY.keys())}"
        )

    # Get the relative path to the URDF file from the registry
    resource_path = ROBOT_URDF_REGISTRY[robot_name]

    # Use pkg_resources to get the absolute path
    urdf_path = pkg_resources.resource_filename("imitation_learning", resource_path)

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    return urdf_path
