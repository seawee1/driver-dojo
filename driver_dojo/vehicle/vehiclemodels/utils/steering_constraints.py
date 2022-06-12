def steering_constraints(steering_angle, steering_velocity, p):
    # steeringConstraints - adjusts the steering velocity based on steering
    # constraints
    #
    # Syntax:  
    #    steeringConstraints(steering_angle,steering_velocity,p)
    #
    # Inputs:
    #    steering_angle - steering angle
    #    steering_velocity - steering velocity
    #    p - steering parameter structure
    #
    # Outputs:
    #    steering_velocity - steering velocity
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

    # steering limit reached?
    if (steering_angle <= p.min and steering_velocity <= 0) or (steering_angle >= p.max and steering_velocity >= 0):
        steering_velocity = 0
    elif steering_velocity <= p.v_min:
        steering_velocity = p.v_min
    elif steering_velocity >= p.v_max:
        steering_velocity = p.v_max

    return steering_velocity

    # ------------- END OF CODE --------------
