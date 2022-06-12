from driver_dojo.vehicle.vehiclemodels.vehicle_parameters import VehicleParameters


def parameters_vehicle4():
    """
    parameters_vehicle4 - parameter set of the kinematic single track model with on-axle trailer (truck model)

    Inputs:
        ---

    Outputs:
        :return p: parameter vector

    Author:         Gerald Würsching
    Written:        21-October-2020
    Last update:    21-October-2020
    Last revision:  ---

    """
    # ------------- BEGIN CODE --------------

    # init vehicle parameters
    p = VehicleParameters()

    # vehicle body dimensions
    p.l = 5.100  # vehicle length [m]
    p.w = 2.550  # vehicle width [m]

    # steering constraints
    p.steering.min = -0.55  # minimum steering angle [rad]
    p.steering.max = 0.55  # maximum steering angle [rad]
    p.steering.v_min = -0.7103  # minimum steering velocity [rad/s]
    p.steering.v_max = 0.7103  # maximum steering velocity [rad/s]

    # longitudinal constraints
    p.longitudinal.v_min = -2.78  # minimum velocity [m/s]  # approximate value: -10 km/h
    p.longitudinal.v_max = 22.22  # minimum velocity [m/s]   # approximate value: 80 km/h
    p.longitudinal.v_switch = 7.824  # switching velocity [m/s]
    p.longitudinal.a_max = 11.5  # maximum absolute acceleration [m/s^2]

    # axes distances
    p.a = 1.8  # distance from spring mass center of gravity to front axle [m]  LENA
    p.b = 1.8  # distance from spring mass center of gravity to rear axle [m]  LENB

    # trailer parameters
    p.trailer.l = 13.6  # trailer length
    p.trailer.w = 2.55  # trailer width
    p.trailer.l_hitch = 12.00  # hitch length
    p.trailer.l_total = 16.5  # total system length
    p.trailer.l_wb = 8.1  # trailer wheelbase

    return p

# ------------- END OF CODE --------------
