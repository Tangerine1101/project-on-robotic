import numpy as np

def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2*np.pi) - np.pi

def IK_fulls_1(T):
    """
    Inverse kinematics
    Only keep solutions where:
    a > 0, b < 0, c > 0
    """
    # Link lengths
    l1, l2, l3, l4, l5, l6 = 170, 200, 220, 45, 0, 0

    # Extract position
    Px = T[0, 3]
    Py = T[1, 3]
    Pz = T[2, 3]

    # Orientation constraint
    theta_14 = np.arctan2(T[0, 1], T[0, 0])

    tol = 1e-9
    Q = []   # store solutions (rad)

    # Base radius
    R = np.sqrt(Px**2 + Py**2)

    # q1 base
    if abs(R) < tol:
        q1_base = 0.0
    else:
        q1_base = np.arctan2(Py, Px)

    # Two branches of q1
    q1_set = [q1_base, q1_base + np.pi]

    for q1 in q1_set:

        # Effective radius
        if abs(wrap_to_pi(q1 - q1_base)) < tol:
            R_eff = +R
        else:
            R_eff = -R

        # Geometry variables
        A = R_eff - l4
        B = Pz - l1 + l5 + l6

        P = np.hypot(A, B)

        # Solve for D
        K = (l3**2 - A**2 - B**2 - l2**2) / (2*l2)
        D = K / P

        if abs(D) > 1 + tol:
            continue

        D = np.clip(D, -1.0, 1.0)

        # q2 branches
        phi   = np.arctan2(B, A)
        alpha = np.arcsin(D)

        q2_set = [phi + alpha, phi + (np.pi - alpha)]

        for q2 in q2_set:

            # Solve q3
            cosq3 = (A + l2*np.sin(q2)) / l3
            sinq3 = (l2*np.cos(q2) - B) / l3

            mag = np.hypot(cosq3, sinq3)
            if abs(mag - 1) > 1e-5:
                continue

            q3 = np.arctan2(sinq3/mag, cosq3/mag)

            # q4 from orientation
            q4 = q1 - theta_14

            # Store (rad)
            Q.append([
                wrap_to_pi(q1),
                wrap_to_pi(q2),
                wrap_to_pi(q3),
                wrap_to_pi(q4)
            ])

    if len(Q) == 0:
        return np.empty((0, 4))

    Q = np.array(Q)

    # Remove duplicates
    Q = np.unique(np.round(Q, 10), axis=0)

    # Convert to degrees
    Q = np.rad2deg(Q)

    # ============================
    # FILTER: a > 0, b < 0, c > 0
    # ============================
    mask = (Q[:, 0] > 0) & (Q[:, 1] < 0) & (Q[:, 2] > 0)
    Q = Q[mask]

    return Q
