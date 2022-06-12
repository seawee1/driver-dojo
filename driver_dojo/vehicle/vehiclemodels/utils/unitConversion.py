def lb_sec2_ft_IN_kg(prev_val):

# 1lb is 4.4482216152605 N
# 1ft is 0.3048 m

    return 4.4482216152605/0.3048*prev_val 


def ft_IN_m(prev_val):
#original: [ft]
#new: [m]

# 1ft is 0.3048 m

    return 0.3048*prev_val 


def lb_ft_sec2_IN_kg_m2(prev_val):

#[kg m^2] = [N m sec^2]

# 1lb is 4.4482216152605 N
# 1ft is 0.3048 m

    return 4.4482216152605*0.3048*prev_val 


def rad_ft_lb_IN_rad_sec2_kg_m2(prev_val):

#original: [rad/(ft lb)]
#new: [rad/(N m)] = [rad s^2/(kg m^2)]

# 1lb is 4.4482216152605 N
# 1ft is 0.3048 m

    return 1/(4.4482216152605*0.3048)*prev_val 


def ft2_IN_m2(prev_val):
#original: [ft^2]
#new: [m^2]

# 1ft is 0.3048 m

    return pow(0.3048, 2)*prev_val 


def lbs_ft_IN_N_m(prev_val):
#original: [lbs/ft]
#new: [N/m]

# 1lbs is 0.45359237 kg
# 1kg is around 9.81 N assuming being close to sea level
# 1ft is 0.3048 m

    return 0.45359237*9.81/0.3048*prev_val 


def lb_sec_ft_IN_N_s_m(prev_val):
#original: [lb sec/ft]
#new: [N sec/m]

# 1lb is 4.4482216152605 N
# 1ft is 0.3048 m

    return 4.4482216152605/0.3048*prev_val 


def ft_lb_rad_IN_N_m_rad(prev_val):
#original: [lb ft/rad]
#new: [N m/rad]

# 1lb is 4.4482216152605 N
# 1ft is 0.3048 m

    return 4.4482216152605*0.3048*prev_val 


def ft_lb_IN_m_N(prev_val):
#original: [ft/lb]
#new: [m/N]

# 1lb is 4.4482216152605 N
# 1ft is 0.3048 m

    return 0.3048/4.4482216152605*prev_val 


def rad_ft_IN_rad_m(prev_val):
#original: [rad/ft]
#new: [rad/m]

# 1ft is 0.3048 m

    return 1/0.3048*prev_val 
