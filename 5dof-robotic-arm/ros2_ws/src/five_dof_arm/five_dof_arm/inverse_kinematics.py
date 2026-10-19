
import numpy as np

# from five_dof_arm.forward_kinematics import end_effector_position
from forward_kinematics import end_effector_position


LOWER_LIMITS = np.array([
    -np.pi, -1.5708, -2.0, -2.0, -np.pi
])

UPPER_LIMITS = np.array([
    np.pi, 1.5708, 2.0, 2.0, np.pi
])


def inverse_kinematics(
    target_position,
    initial_guess=None,
    max_iterations=1000,
    tolerance=1e-5,
    damping=0.03,
):
    """
    Solve position-only IK using damped least squares.

    Args:
        target_position: Desired [x, y, z] in metres.
        initial_guess: Initial five joint angles in radians.
        max_iterations: Maximum optimization iterations.
        tolerance: Acceptable position error in metres.
        damping: Damping factor for numerical stability.

    Returns:
        (joint_angles, success, position_error)

        joint_angles: Five angles in radians.
        success: Whether the target was reached within tolerance.
        position_error: Final Euclidean position error in metres.
    """
    target = np.asarray(target_position, dtype=float)

    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("Target must contain three finite coordinates.")

    if initial_guess is None:
        q = np.array([0.0, 0.3, -0.6, 0.3, 0.0])
    else:
        q = np.asarray(initial_guess, dtype=float).copy()

    if q.shape != (5,) or not np.all(np.isfinite(q)):
        raise ValueError("Initial guess must contain five finite angles.")

    q = np.clip(q, LOWER_LIMITS, UPPER_LIMITS)

    step_size = 1e-6

    for _ in range(max_iterations):
        current = end_effector_position(q)
        error = target - current
        error_norm = np.linalg.norm(error)

        if error_norm < tolerance:
            return q, True, error_norm

        # Numerical position Jacobian: 3 rows x 5 columns.
        J = np.zeros((3, 5))

        for j in range(5):
            q_perturbed = q.copy()
            q_perturbed[j] += step_size

            J[:, j] = (
                end_effector_position(q_perturbed) - current
            ) / step_size

        # Damped least-squares joint update.
        A = J @ J.T + damping**2 * np.eye(3)
        dq = J.T @ np.linalg.solve(A, error)

        # Limit each iteration to avoid excessively large updates.
        largest_step = np.max(np.abs(dq))
        if largest_step > 0.15:
            dq *= 0.15 / largest_step

        q = np.clip(q + dq, LOWER_LIMITS, UPPER_LIMITS)

    final_error = np.linalg.norm(
        target - end_effector_position(q)
    )

    return q, final_error < tolerance, final_error


if __name__ == "__main__":
    target = [0.10, 0.0, 0.25]  # pen-tip position in metres

    q, success, error = inverse_kinematics(target)

    print("Success:", success)
    print("Joint angles (degrees):", np.rad2deg(q))
    print("Pen-tip position:", end_effector_position(q))
    print("Error (mm):", error * 1000)