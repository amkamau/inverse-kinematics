
import numpy as np


PEN_LENGTH = 0.080  # metres
PEN_ANGLE = np.deg2rad(45.0)


def rotation_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ], dtype=float)


def rotation_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c,  0, s, 0],
        [0,  1, 0, 0],
        [-s, 0, c, 0],
        [0,  0, 0, 1],
    ], dtype=float)


def translation_z(distance):
    T = np.eye(4)
    T[2, 3] = distance
    return T


def forward_kinematics(joint_angles):
    """
    Return the homogeneous transform of the PEN TIP.

    Args:
        joint_angles: [q1, q2, q3, q4, q5], radians.

    Returns:
        4x4 transformation matrix.
        Position is in metres, orientation is a rotation matrix.
    """
    if len(joint_angles) != 5:
        raise ValueError("Expected exactly five joint angles.")

    q1, q2, q3, q4, q5 = map(float, joint_angles)

    # Base to gripper/link4 frame using URDF joint origins.
    T = (
        translation_z(0.037)
        @ rotation_z(q1)
        @ translation_z(0.028)
        @ rotation_y(q2)
        @ translation_z(0.101)
        @ rotation_y(q3)
        @ translation_z(0.095)
        @ rotation_y(q4)
        @ translation_z(0.068)
        @ rotation_z(q5)
    )

    # Pen axis is tilted 45 degrees from downward vertical
    # in the gripper's local frame.
    #
    # Rotate local +Z to point downwards and sideways,
    # then translate 80 mm along the pen's own axis.
    T_pen = (
        rotation_y(np.pi - PEN_ANGLE)
        @ translation_z(PEN_LENGTH)
    )

    return T @ T_pen


def end_effector_position(joint_angles):
    """Return pen-tip coordinates [x, y, z] in metres."""
    return forward_kinematics(joint_angles)[:3, 3].copy()


if __name__ == "__main__":
    q = np.zeros(5)

    T = forward_kinematics(q)

    print("Pen-tip transformation:")
    print(T)

    print("Pen-tip position [x, y, z] in metres:")
    print(T[:3, 3])

    print("Pen-tip position [x, y, z] in mm:")
    print(T[:3, 3] * 1000)
