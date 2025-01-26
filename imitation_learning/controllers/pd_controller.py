import numpy as np

class PDController:
    def __init__(self, robot, p_gain=5.0, d_gain=0.2):
        """
        Initialize the PD controller.

        Parameters:
            robot (PybulletRobot): An instance of the PybulletRobot class.
            p_gain (float): Proportional gain.
            d_gain (float): Derivative gain.
        """
        self.robot = robot
        self.P_GAIN = p_gain
        self.D_GAIN = d_gain

    def __call__(self, desired_pos, desired_vel):
        """
        Calculate joint velocities to track the desired end-effector position and velocity.

        Parameters:
            desired_pos (np.ndarray): Desired end-effector position (3D).
            desired_vel (np.ndarray): Desired end-effector velocity (3D).

        Returns:
            np.ndarray: Joint velocities (`q_dot`) to achieve the desired motion.
        """
        # Get the current end-effector position
        current_pos = self.robot.forward_kinematics()

        # Calculate the Jacobian matrix (translational part)
        jacobian_matrix, _, _ = self.robot.jacobian()

        # Compute the pseudo-inverse of the Jacobian
        pinv_jacobian = np.linalg.pinv(jacobian_matrix)

        # Calculate the current end-effector velocity (based on position difference)
        joint_positions, joint_velocities, _ = self.robot.get_joint_states()
        current_vel = np.array(joint_velocities, dtype=np.float32)  # Joint-space velocity

        # PD control error
        pos_error = desired_pos - current_pos
        vel_error = desired_vel - np.matmul(jacobian_matrix, current_vel)

        # PD control signal
        pd_error = self.P_GAIN * pos_error + self.D_GAIN * vel_error

        # Compute joint velocities
        q_dot = np.matmul(pinv_jacobian, pd_error)

        return q_dot
