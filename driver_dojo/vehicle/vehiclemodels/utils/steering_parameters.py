class SteeringParameters():
    def __init__(self):
        #constraints regarding steering
        self.min = []  #minimum steering angle [rad]
        self.max = []  #maximum steering angle [rad]
        self.v_min = []  #minimum steering velocity [rad/s]
        self.v_max = []  #maximum steering velocity [rad/s]
