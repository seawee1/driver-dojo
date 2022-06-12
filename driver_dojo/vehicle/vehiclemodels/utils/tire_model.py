import math


# sign function
def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


# longitudinal tire forces
def formula_longitudinal(kappa, gamma, F_z, p):
    # turn slip is neglected, so xi_i=1
    # all scaling factors lambda = 1

    # coordinate system transformation
    kappa = -kappa

    S_hx = p.p_hx1
    S_vx = F_z * p.p_vx1

    kappa_x = kappa + S_hx
    mu_x = p.p_dx1 * (1 - p.p_dx3 * gamma ** 2)

    C_x = p.p_cx1
    D_x = mu_x * F_z
    E_x = p.p_ex1
    K_x = F_z * p.p_kx1
    B_x = K_x / (C_x * D_x)

    # magic tire formula
    return D_x * math.sin(C_x * math.atan(B_x * kappa_x - E_x * (B_x * kappa_x - math.atan(B_x * kappa_x))) + S_vx)


# lateral tire forces
def formula_lateral(alpha, gamma, F_z, p):
    # turn slip is neglected, so xi_i=1
    # all scaling factors lambda = 1

    # coordinate system transformation
    # alpha = -alpha

    S_hy = sign(gamma) * (p.p_hy1 + p.p_hy3 * math.fabs(gamma))
    S_vy = sign(gamma) * F_z * (p.p_vy1 + p.p_vy3 * math.fabs(gamma))

    alpha_y = alpha + S_hy
    mu_y = p.p_dy1 * (1 - p.p_dy3 * gamma ** 2)

    C_y = p.p_cy1
    D_y = mu_y * F_z
    E_y = p.p_ey1
    K_y = F_z * p.p_ky1  # simplify K_y0 to p.p_ky1*F_z
    B_y = K_y / (C_y * D_y)

    # magic tire formula
    F_y = D_y * math.sin(C_y * math.atan(B_y * alpha_y - E_y * (B_y * alpha_y - math.atan(B_y * alpha_y)))) + S_vy

    res = []
    res.append(F_y)
    res.append(mu_y)
    return res


# longitudinal tire forces for combined slip
def formula_longitudinal_comb(kappa, alpha, F0_x, p):
    # turn slip is neglected, so xi_i=1
    # all scaling factors lambda = 1

    S_hxalpha = p.r_hx1

    alpha_s = alpha + S_hxalpha

    B_xalpha = p.r_bx1 * math.cos(math.atan(p.r_bx2 * kappa))
    C_xalpha = p.r_cx1
    E_xalpha = p.r_ex1
    D_xalpha = F0_x / (math.cos(C_xalpha * math.atan(
        B_xalpha * S_hxalpha - E_xalpha * (B_xalpha * S_hxalpha - math.atan(B_xalpha * S_hxalpha)))))

    # magic tire formula
    return D_xalpha * math.cos(
        C_xalpha * math.atan(B_xalpha * alpha_s - E_xalpha * (B_xalpha * alpha_s - math.atan(B_xalpha * alpha_s))))


# lateral tire forces for combined slip
def formula_lateral_comb(kappa, alpha, gamma, mu_y, F_z, F0_y, p):
    # turn slip is neglected, so xi_i=1
    # all scaling factors lambda = 1

    S_hykappa = p.r_hy1

    kappa_s = kappa + S_hykappa

    B_ykappa = p.r_by1 * math.cos(math.atan(p.r_by2 * (alpha - p.r_by3)))
    C_ykappa = p.r_cy1
    E_ykappa = p.r_ey1
    D_ykappa = F0_y / (math.cos(C_ykappa * math.atan(
        B_ykappa * S_hykappa - E_ykappa * (B_ykappa * S_hykappa - math.atan(B_ykappa * S_hykappa)))))

    D_vykappa = mu_y * F_z * (p.r_vy1 + p.r_vy3 * gamma) * math.cos(math.atan(p.r_vy4 * alpha))
    S_vykappa = D_vykappa * math.sin(p.r_vy5 * math.atan(p.r_vy6 * kappa))

    # magic tire formula
    return D_ykappa * math.cos(C_ykappa * math.atan(
        B_ykappa * kappa_s - E_ykappa * (B_ykappa * kappa_s - math.atan(B_ykappa * kappa_s)))) + S_vykappa
