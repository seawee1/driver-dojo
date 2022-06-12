class LongitudinalParameters:
    def __init__(self):
        # constraints regarding longitudinal dynamics
        self.v_min = None  # minimum velocity [m/s]
        self.v_max = None  # minimum velocity [m/s]
        self.v_switch = None  # switching velocity [m/s]
        self.a_max = None  # maximum absolute acceleration [m/s^2]
