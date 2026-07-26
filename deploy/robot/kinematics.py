"""Inverse kinematics for the robot arm.

Copied as-is from the ROS2-era ros2/src/robot_pkg/pkg/FWK_Degree.py — the math is
not ROS-specific and is unchanged by the de-ROSification. Link lengths match
binh.m. The q4 (+90) shift here converts the IK solution into the servo's
0-180 degree frame; callers must NOT add another +90 on top of this (that used
to happen in planner_node.py's send_arm_goal and is bug 1.6 in doc/bug-report.md
-- fixed by removing the duplicate offset in planner.py instead of here).
"""
import numpy as np


def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def solve_ik(x, y, z):
    """
    Inverse Kinematics with Safety Limits & Calibration
    Link lengths match binh.m (Corrected Hardware)
    """
    # ============================
    #     CALIBRATION OFFSETS
    # ============================
    # Adjust these to shift the "World (0,0,0)" relative to the Robot Base
    OFFSET_X = 20.0   # mm
    OFFSET_Y = 20.0   # mm
    OFFSET_Z = 28.0   # mm

    # Link lengths (mm)
    l1, l2, l3, l4, l5, l6 = 180, 200, 220, 50, 15, 100

    T = np.array([
        [1.0, 0.0, 0.0, float(x)],
        [0.0, -1.0, 0.0, float(y)],
        [0.0, 0.0, -1.0, float(z)],
        [0.0, 0.0, 0.0, 1.0],
    ])

    # Extract pose and apply offset
    Px = T[0, 3] + OFFSET_X
    Py = T[1, 3] + OFFSET_Y
    Pz = T[2, 3] + OFFSET_Z

    # Orientation constraint
    theta_14 = np.arctan2(T[0, 1], T[0, 0])

    tol = 1e-9
    Q = []   # store solutions

    # Base radius
    R = np.hypot(Px, Py)

    # q1 base
    if abs(R) < tol:
        q1_base = 0.0
    else:
        q1_base = np.arctan2(Py, Px)

    # Two branches for q1
    q1_set = [q1_base, q1_base + np.pi]

    for q1 in q1_set:
        # Effective radius check
        if abs(wrap_to_pi(q1 - q1_base)) < tol:
            R_eff = R
        else:
            R_eff = -R

        # Geometry variables
        A = R_eff - l4
        B = Pz - l1 + l5 + l6
        P = np.hypot(A, B)

        if P < tol:
            continue

        # Solve for D
        num = l3**2 - A**2 - B**2 - l2**2
        den = 2 * l2
        K = num / den
        D = K / P

        if abs(D) > 1 + tol:
            continue
        D = np.clip(D, -1.0, 1.0)

        # q2 branches
        phi = np.arctan2(B, A)
        alpha = np.arcsin(D)
        q2_set = [phi + alpha, phi + (np.pi - alpha)]

        for q2 in q2_set:
            # Solve q3
            cosq3 = (A + l2 * np.sin(q2)) / l3
            sinq3 = (l2 * np.cos(q2) - B) / l3

            mag = np.hypot(cosq3, sinq3)
            if abs(mag - 1) > 1e-5:
                continue

            q3 = np.arctan2(sinq3, cosq3)

            # Solve q4
            q4 = q1 - theta_14

            Q.append([
                wrap_to_pi(q1),
                wrap_to_pi(q2),
                wrap_to_pi(q3),
                wrap_to_pi(q4),
            ])

    if len(Q) == 0:
        return np.empty((0, 4))

    Q = np.array(Q)
    Q = np.unique(np.round(Q, 10), axis=0)
    Q = np.rad2deg(Q)

    # ============================
    #       SAFETY FILTER
    # ============================
    # 1. 16 > q1 > -90 (Base limitation)
    mask_q1 = (Q[:, 0] > -90) & (Q[:, 0] < 16)

    # 2. -80 < q2 < 0 (Shoulder limitation)
    mask_q2 = (Q[:, 1] > -80) & (Q[:, 1] < 60)

    # 3. 90 > q3 > 0 (Elbow limitation)
    mask_q3 = (Q[:, 2] > 0) & (Q[:, 2] < 90)

    # 4. -90 < q4 < 90 (Servo Limitation), shifted into the 0-180 servo frame
    temp = Q[:, 3] + 90
    Q[:, 3] = temp
    mask_q4 = (Q[:, 3] > 0) & (Q[:, 3] < 180)
    # 5. Collision Check: q2 + q3 < 60
    mask_col = (Q[:, 1] + Q[:, 2]) < 60

    # Apply all masks
    valid_indices = mask_q1 & mask_q2 & mask_q3 & mask_q4 & mask_col
    Q_valid = Q[valid_indices]

    if len(Q_valid) == 0:
        return np.empty((0, 4))  # No safe solution

    # Return only the FIRST valid solution
    return Q_valid[:1]


def solve_ik_first(x, y, z):
    """Convenience wrapper: returns a plain list [q1,q2,q3,q4] in degrees
    (servo frame for q4), or None if unreachable."""
    sol = solve_ik(x, y, z)
    if sol.size == 0:
        return None
    return sol[0].tolist()
