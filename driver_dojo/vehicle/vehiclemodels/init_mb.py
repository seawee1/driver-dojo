import math


def init_mb(init_state, p):
    # init_MB - generates the initial state vector for the multi-body model
    #
    # Syntax:  
    #     x0 = init_MB(init_state, p)
    #
    # Inputs:
    #     init_state - core initial states
    #     p - parameter vector
    #
    # Outputs:
    #     x0 - initial state vector
    #
    # Example: 
    #
    # See also: ---

    # Author:       Matthias Althoff
    # Written:      11-January-2017
    # Last update:  ---
    # Last revision:---

    # ------------- BEGIN CODE --------------

    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity in x-direction
    # x5 = yaw angle
    # x6 = yaw rate

    # x7 = roll angle
    # x8 = roll rate
    # x9 = pitch angle
    # x10 = pitch rate
    # x11 = velocity in y-direction
    # x12 = z-position
    # x13 = velocity in z-direction

    # x14 = roll angle front
    # x15 = roll rate front
    # x16 = velocity in y-direction front
    # x17 = z-position front
    # x18 = velocity in z-direction front

    # x19 = roll angle rear
    # x20 = roll rate rear
    # x21 = velocity in y-direction rear
    # x22 = z-position rear
    # x23 = velocity in z-direction rear

    # x24 = left front wheel angular speed
    # x25 = right front wheel angular speed
    # x26 = left rear wheel angular speed
    # x27 = right rear wheel angular speed

    # x28 = delta_y_f
    # x29 = delta_y_r

    # u1 = steering angle velocity of front wheels
    # u2 = acceleration

    # obtain initial states from vector
    sx0 = init_state[0]
    sy0 = init_state[1]
    delta0 = init_state[2]
    vel0 = init_state[3]
    Psi0 = init_state[4]
    dotPsi0 = init_state[5]
    beta0 = init_state[6]

    # create equivalent bicycle parameters
    g = 9.81  # [m/s^2]

    # auxiliary initial states
    F0_z_f = p.m_s * g * p.b / ((p.a + p.b)) + p.m_uf * g
    F0_z_r = p.m_s * g * p.a / ((p.a + p.b)) + p.m_ur * g

    # sprung mass states
    x0 = []  # init initial state vector
    x0.append(sx0)  # x-position in a global coordinate system
    x0.append(sy0)  # y-position in a global coordinate system
    x0.append(delta0)  # steering angle of front wheels
    x0.append(math.cos(beta0) * vel0)  # velocity in x-direction
    x0.append(Psi0)  # yaw angle
    x0.append(dotPsi0)  # yaw rate
    x0.append(0)  # roll angle
    x0.append(0)  # roll rate
    x0.append(0)  # pitch angle
    x0.append(0)  # pitch rate
    x0.append(math.sin(beta0) * vel0)  # velocity in y-direction
    x0.append(0)  # z-position (zero height corresponds to steady state solution)
    x0.append(0)  # velocity in z-direction

    # unsprung mass states (front)
    x0.append(0)  # roll angle front
    x0.append(0)  # roll rate front
    x0.append(math.sin(beta0) * vel0 + p.a * dotPsi0)  # velocity in y-direction front
    x0.append((F0_z_f) / (2 * p.K_zt))  # z-position front
    x0.append(0)  # velocity in z-direction front

    # unsprung mass states (rear)
    x0.append(0)  # roll angle rear
    x0.append(0)  # roll rate rear
    x0.append(math.sin(beta0) * vel0 - p.b * dotPsi0)  # velocity in y-direction rear
    x0.append((F0_z_r) / (2 * p.K_zt))  # z-position rear
    x0.append(0)  # velocity in z-direction rear

    # wheel states
    x0.append(x0[3] / (p.R_w))  # left front wheel angular speed
    x0.append(x0[3] / (p.R_w))  # right front wheel angular speed
    x0.append(x0[3] / (p.R_w))  # left rear wheel angular speed
    x0.append(x0[3] / (p.R_w))  # right rear wheel angular speed

    x0.append(0)  # delta_y_f
    x0.append(0)  # delta_y_r

    return x0

    # ------------- END OF CODE --------------
