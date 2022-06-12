def acceleration_constraints(velocity, acceleration, p):
    # accelerationConstraints - adjusts the acceleration based on acceleration
    # constraints
    #
    # Syntax:  
    #    accelerationConstraints(velocity,acceleration,p)
    #
    # Inputs:
    #    acceleration - acceleration in driving direction
    #    velocity - velocity in driving direction
    #    p - longitudinal parameter structure
    #
    # Outputs:
    #    acceleration - acceleration in driving direction
    #
    # Example: 
    #
    # Other m-files required: none
    # Subfunctions: none
    # MAT-files required: none
    #
    # See also: ---

    # Author:       Matthias Althoff
    # Written:      15-December-2017
    # Last update:  ---
    # Last revision:---

    # ------------- BEGIN CODE --------------

    # positive acceleration limit
    if velocity > p.v_switch:
        posLimit = p.a_max * p.v_switch / velocity
    else:
        posLimit = p.a_max

    # acceleration limit reached?
    if (velocity <= p.v_min and acceleration <= 0) or (velocity >= p.v_max and acceleration >= 0):
        # print("accleration doesn't reach the limit")
        acceleration = 0
    elif acceleration <= -p.a_max:
        # print("acceleration <= -p.a_max")
        acceleration = -p.a_max
    elif acceleration >= posLimit:
        # print("acceleration >= posLimit")
        acceleration = posLimit
        acceleration = posLimit

    return acceleration

    # ------------- END OF CODE --------------
