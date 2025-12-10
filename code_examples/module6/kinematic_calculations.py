import numpy as np

# This is a conceptual script for kinematic calculations for a simplified
# 2-DOF robotic arm (similar to a humanoid arm for demonstration).

def forward_kinematics(link_lengths, joint_angles):
    """
    Conceptual Forward Kinematics for a 2-DOF arm.
    Calculates the end-effector position given link lengths and joint angles.
    Args:
        link_lengths (list): [L1, L2] lengths of the two links.
        joint_angles (list): [theta1, theta2] in radians.
    Returns:
        tuple: (x, y) coordinates of the end-effector.
    """
    l1, l2 = link_lengths
    theta1, theta2 = joint_angles

    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    
    return x, y

def inverse_kinematics_2dof(link_lengths, target_pos):
    """
    Conceptual Inverse Kinematics for a 2-DOF arm.
    Calculates joint angles to reach a target (x, y) position.
    This is a simplified solution for a planar arm.
    Args:
        link_lengths (list): [L1, L2] lengths of the two links.
        target_pos (list): [x, y] coordinates of the target.
    Returns:
        list of tuples: Possible joint angle solutions [(theta1, theta2)] or None if unreachable.
    """
    l1, l2 = link_lengths
    x, y = target_pos

    # Calculate squared distance from origin to target
    d_squared = x**2 + y**2

    # Check if target is reachable
    if d_squared > (l1 + l2)**2 or d_squared < (l1 - l2)**2:
        print(f"Target ({x:.2f}, {y:.2f}) unreachable with links {l1}, {l2}.")
        return None

    # Calculate theta2 (Elbow angle)
    cos_theta2 = (d_squared - l1**2 - l2**2) / (2 * l1 * l2)
    # Due to floating point inaccuracies, clamp cos_theta2 to [-1, 1]
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0) 
    
    theta2_1 = np.arccos(cos_theta2)
    theta2_2 = -theta2_1 # Two possible solutions for elbow (elbow up/down)

    # Calculate theta1 (Shoulder angle)
    k1 = l1 + l2 * np.cos(theta2_1)
    k2 = l2 * np.sin(theta2_1)
    theta1_1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    k3 = l1 + l2 * np.cos(theta2_2)
    k4 = l2 * np.sin(theta2_2)
    theta1_2 = np.arctan2(y, x) - np.arctan2(k4, k3)
    
    # Return in degrees for readability in print
    return [
        (np.degrees(theta1_1), np.degrees(theta2_1)),
        (np.degrees(theta1_2), np.degrees(theta2_2))
    ]

if __name__ == '__main__':
    link_lengths = [1.0, 1.0] # L1=1m, L2=1m

    # --- Forward Kinematics Example ---
    joint_angles_rad = [np.radians(30), np.radians(60)] # 30 deg, 60 deg
    x_fk, y_fk = forward_kinematics(link_lengths, joint_angles_rad)
    print(f"FK: Joint Angles (deg): {np.degrees(joint_angles_rad[0]):.2f}, {np.degrees(joint_angles_rad[1]):.2f} -> End-effector Pos: ({x_fk:.2f}, {y_fk:.2f})")

    # --- Inverse Kinematics Example ---
    target_pos_ik = [1.5, 0.5]
    solutions_ik = inverse_kinematics_2dof(link_lengths, target_pos_ik)

    if solutions_ik:
        print(f"\nIK: Target Position: ({target_pos_ik[0]:.2f}, {target_pos_ik[1]:.2f})")
        for i, (theta1, theta2) in enumerate(solutions_ik):
            print(f"  Solution {i+1}: Theta1={theta1:.2f} deg, Theta2={theta2:.2f} deg")
    
        # Verify one solution using FK
        if solutions_ik:
            test_angles_rad = [np.radians(solutions_ik[0][0]), np.radians(solutions_ik[0][1])]
            x_test, y_test = forward_kinematics(link_lengths, test_angles_rad)
            print(f"  (Verification with FK) -> End-effector Pos: ({x_test:.2f}, {y_test:.2f})")

    # --- Unreachable Target Example ---
    unreachable_target = [3.0, 0.0] # Outside reach
    inverse_kinematics_2dof(link_lengths, unreachable_target)
