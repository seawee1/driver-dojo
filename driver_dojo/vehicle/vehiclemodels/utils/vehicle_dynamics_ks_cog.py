from driver_dojo.vehicle.vehiclemodels.utils.acceleration_constraints import acceleration_constraints
from driver_dojo.vehicle.vehiclemodels.utils.steering_constraints import steering_constraints
import math


def vehicle_dynamics_ks_cog(x, u_init, p):
    """
    vehicle_dynamics_ks_cog - kinematic single-track vehicle dynamics
    reference point: center of mass

    Inputs:
        :param x: vehicle state vector
        :param u_init: vehicle input vector
        :param p: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author: Gerald Würsching
    Written: 17-November-2020
    Last update: 17-November-2020
    Last revision: ---
    """
    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity at center of mass
    # x5 = yaw angle

    # wheelbase
    l_wb = p.a + p.b

    # consider steering constraints
    u = []
    u.append(steering_constraints(x[2], u_init[0], p.steering))  # different name u_init/u due to side effects of u
    # consider acceleration constraints
    u.append(
        acceleration_constraints(x[3], u_init[1], p.longitudinal))  # different name u_init/u due to side effects of u

    # slip angle (beta) from vehicle kinematics
    beta = math.atan(math.tan(x[2]) * p.b/l_wb)

    # system dynamics
    f = [x[3] * math.cos(beta + x[4]),
         x[3] * math.sin(beta + x[4]),
         u[0],
         u[1],
         x[3] * math.cos(beta) * math.tan(x[2]) / l_wb]

    return f
