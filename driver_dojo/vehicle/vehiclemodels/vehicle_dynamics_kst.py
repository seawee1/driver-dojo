import math

from driver_dojo.vehicle.vehiclemodels.utils.steering_constraints import steering_constraints
from driver_dojo.vehicle.vehiclemodels.utils.acceleration_constraints import acceleration_constraints

__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2020a"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"


def vehicle_dynamics_kst(x, u_init, p):
    """
    vehicle_dynamics_kst - kinematic single-track with one on-axle trailer vehicle dynamics
    reference point: rear axle

    Syntax:
        f = vehicle_dynamics_kst(x,u,p)

    Inputs:
        :param x: vehicle state vector
        :param u_init: vehicle input vector
        :param p: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author:         Gerald Würsching
    Written:        21-October-2020
    Last update:    21-October-2020
    Last revision:  ---
    """

    # ------------- BEGIN CODE --------------

    # create equivalent kinematic single-track parameters
    l_wb = p.a + p.b        # wheel base
    l_wbt = p.trailer.l_wb  # wheel base trailer

    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity in x-direction
    # x5 = yaw angle
    # x6 = hitch angle

    # inputs
    # u1 = steering angle velocity of front wheels
    # u2 = longitudinal acceleration

    u = []
    # consider steering constraints
    u.append(steering_constraints(x[2], u_init[0], p.steering))  # different name uInit/u due to side effects of u
    # consider acceleration constraints
    u.append(acceleration_constraints(x[3], u_init[1], p.longitudinal))  # different name uInit/u due to side effects of u

    # hitch angle constraints
    if -math.pi/2 <= x[5] <= math.pi/2:
        d_alpha = -x[3] * (math.sin(x[5]) / l_wbt + math.tan(x[2]) / l_wb)
    else:
        d_alpha = 0
        x[5] = -math.pi/2 if x[5] < -math.pi/2 else math.pi/2

    # system dynamics
    f = [x[3] * math.cos(x[4]),
         x[3] * math.sin(x[4]),
         u[0],
         u[1],
         x[3] / l_wb * math.tan(x[2]),
         d_alpha]

    return f

    # ------------- END OF CODE --------------
