import importlib.resources as pkg_resources
import os

# Dictionary to store URDF paths for different robot models
ROBOT_URDF_REGISTRY = {
    "mycobot": "imitation_learning.robot_models.mycobot_description.urdf/mycobot_urdf.urdf",
    # You can add more robots here in the format:
    # "robot_name": "package.subpackage.path_to_urdf/urdf_file_name"
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

    Example:
        # To get the URDF path for MyCobot:
        urdf_path = get_urdf_path("mycobot")

        # To add a new robot (example):
        ROBOT_URDF_REGISTRY["new_robot"] = "imitation_learning.robot_models.new_robot_description.urdf/new_robot_urdf.urdf"
    """
    if robot_name not in ROBOT_URDF_REGISTRY:
        raise ValueError(
            f"Robot name '{robot_name}' not found in the registry. "
            f"Available robots: {list(ROBOT_URDF_REGISTRY.keys())}"
        )

    # Get the resource path from the registry
    resource_path = ROBOT_URDF_REGISTRY[robot_name]

    # Extract the package and file from the resource path
    package, file_name = resource_path.rsplit("/", 1)

    # Use importlib.resources to get the full path
    with pkg_resources.path(package, file_name) as urdf_path:
        return str(urdf_path)
