import math


def init_std(init_state, p):
    """
    init_std generates the initial state vector for the drift single track model

    Syntax:
        x0 = init_std(init_state, p)

    Inputs:
        :param init_state: core initial states
        :param p: parameter vector

    Outputs:
        :return x0: initial state vector

    Author:         Gerald Würsching
    Written:        23-October-2020
    Last update:    23-October-2020
    Last revision:  ---
    """

    # ------------- BEGIN CODE --------------
    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity at vehicle center
    # x5 = yaw angle
    # x6 = yaw rate
    # x7 = slip angle at vehicle center
    # x8 = front wheel angular speed
    # x9 = rear wheel angular speed

    # create initial state vector
    x0 = init_state.copy()
    x0.append(x0[3]*math.cos(x0[6])/(math.cos(x0[2])*p.R_w))  # init front wheel angular speed
    x0.append(x0[3]*math.cos(x0[6])/p.R_w)  # init rear wheel angular speed

    return x0

    # ------------- END OF CODE --------------
