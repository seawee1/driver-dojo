def init_kst(init_state, alpha0):
    """
    init_KST - generates the initial state vector for the kinematic single-track model with on-axle trailer

    Syntax:
        x0 = init_KST(init_state, p)

    Inputs:
        :param init_state: core initial states
        :param alpha0: initial hitch angle

    Outputs:
        :return x0: initial state vector

    Author:         Gerald Würsching
    Written:        21-October-2020
    Last update:    21-October-2020
    Last revision:  ---
    """

    # ------------- BEGIN CODE --------------
    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity in x-direction
    # x5 = yaw angle
    # x6 = hitch angle

    # obtain initial states from vector
    sx0 = init_state[0]
    sy0 = init_state[1]
    delta0 = init_state[2]
    vel0 = init_state[3]
    Psi0 = init_state[4]

    x0 = [sx0, sy0, delta0, vel0, Psi0, alpha0]

    return x0

    # ------------- END OF CODE --------------
