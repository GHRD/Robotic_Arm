import numpy as np

# class RoboticArm:
#     def __init__(self, joint_angles=np.zeros(6), joint_velocities=np.zeros(6)):
#         self.joint_angles = joint_angles
#         self.joint_velocities = joint_velocities
#         self.link_lengths = [1, 1, 1, 1, 1, 1]  # lengths of the arm segments
#         self.link_offsets = [0, 0, 0, 0, 0, 0]  # offsets between the joint axes

#     def forward_kinematics(self):
#         # Homogeneous transformation matrices for each joint
#         T_01 = self.DH_matrix(self.joint_angles[0], self.link_lengths[0], self.link_offsets[0], -np.pi/2)
#         T_12 = self.DH_matrix(self.joint_angles[1], self.link_lengths[1], self.link_offsets[1], 0)
#         T_23 = self.DH_matrix(self.joint_angles[2], self.link_lengths[2], self.link_offsets[2], -np.pi/2)
#         T_34 = self.DH_matrix(self.joint_angles[3], self.link_lengths[3], self.link_offsets[3], np.pi/2)
#         T_45 = self.DH_matrix(self.joint_angles[4], self.link_lengths[4], self.link_offsets[4], -np.pi/2)
#         T_56 = self.DH_matrix(self.joint_angles[5], self.link_lengths[5], self.link_offsets[5], 0)

#         # End-effector position in the base frame
#         T_06 = T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56
#         pos = T_06[:3, 3]
#         return pos
class RoboticArm:
    def __init__(self, joint_angles=np.zeros(3), joint_velocities=np.zeros(3)):
        self.joint_angles = joint_angles
        self.joint_velocities = joint_velocities
        self.link_lengths = [1, 1, 1]  # lengths of the arm segments
        self.link_offsets = [0, 0, 0]  # offsets between the joint axes

    def forward_kinematics(self):
        # Homogeneous transformation matrices for each joint
        T_01 = self.DH_matrix(self.joint_angles[0], self.link_lengths[0], self.link_offsets[0], -np.pi/2)
        T_12 = self.DH_matrix(self.joint_angles[1], self.link_lengths[1], self.link_offsets[1], 0)
        T_23 = self.DH_matrix(self.joint_angles[2], self.link_lengths[2], self.link_offsets[2], -np.pi/2)

        # End-effector position in the base frame
        T_03 = T_01 @ T_12 @ T_23
        pos = T_03[:3, 3]
        return pos

    def DH_matrix(self, theta, d, a, alpha):
        # Homogeneous transformation matrix for a DH parameter set
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return T

    def inverse_kinematics(self, pos, threshold=1e-4, max_iterations=1000):
        # Implement inverse kinematics algorithm to solve for joint angles given end-effector position
        # using the Jacobian transpose method
        
        # Initialize joint angles
        q = self.joint_angles.copy()
        
        # Set the maximum step size for each iteration
        step_size = 0.01
        
        # Set the desired end-effector position
        pos_d = pos.copy()
        
        # Set the threshold for the error between the current and desired end-effector position
        error_threshold = threshold
        
        # Set the maximum number of iterations
        max_iter = max_iterations
    def jacobian(self, joint_angles):
        end_effector_pos, J = self.forward_kinematics(joint_angles)
        return J
        # Define the Jacobian transpose function
    def J_T(self, q):
            # Calculate the Jacobian matrix
        J = self.jacobian(q)
            
            # Calculate the transpose of the Jacobian matrix
        J_T = np.transpose(J)
            
        return J_T
        
        # Define the error function
    def error_function(self, goal_pos):
            # Compute the current end-effector position
        current_pos = self.forward_kinematics()

            # Compute the error as the difference between the current position and the goal position
        error = goal_pos - current_pos

            # Return the error vector
        return error
# Define the update_joints_angles function
    def update_joint_angles(self, dt):
        # Update joint angles using forward kinematics and the Jacobian transpose method
        # Set the maximum step size for each iteration
        step_size = 0.01

        # Set the desired end-effector position
        pos_d = self.forward_kinematics()

        # Set the threshold for the error between the current and desired end-effector position
        error_threshold = 1e-4

        # Set the maximum number of iterations
        max_iter = 1000

        # Initialize the error to a large value
        error = np.inf

        # Initialize the iteration counter
        i = 0

        # Repeat until the error is less than the threshold or the maximum number of iterations is reached
        while np.linalg.norm(error) > error_threshold and i < max_iter:
            # Calculate the Jacobian transpose
            J_T = self.J_T(self.joint_angles)

            # Calculate the error between the current and desired end-effector positions
            error = error_function(self, pos_d)

            # Calculate the step to update the joint angles
            delta_q = J_T @ error * step_size

            # Update the joint angles
            self.joint_angles += delta_q

            # Increment the iteration counter
            i += 1
    def update_joint_angles(self, control_output, dt):
    # Update joint angles using forward kinematics and the Jacobian transpose method
    # Set the maximum step size for each iteration
        step_size = 0.01

    # Set the desired end-effector position
        target_pos = self.forward_kinematics()

    # Set the threshold for the error between the current and desired end-effector position
        error_threshold = 1e-4

    # Set the maximum number of iterations
        max_iter = 1000

    # Initialize the error to a large value
        error = np.inf

    # Initialize the iteration counter
        i = 0

    # Repeat until the error is less than the threshold or the maximum number of iterations is reached
        while np.linalg.norm(error) > error_threshold and i < max_iter:
        # Calculate the Jacobian transpose
            J_T = self.J_T(self.joint_angles)

        # Calculate the error between the current and desired end-effector positions
            error = target_pos - self.forward_kinematics(self.joint_angles)

        # Calculate the step to update the joint angles
            delta_q = J_T @ control_output * step_size

        # Update the joint angles
            self.joint_angles += delta_q

        # Increment the iteration counter
        i += 1
