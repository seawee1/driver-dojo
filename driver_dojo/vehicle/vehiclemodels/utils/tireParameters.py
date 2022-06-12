class TireParameters():
    def __init__(self):
        # tire parameters from ADAMS handbook
        # longitudinal coefficients
        self.p_cx1 = None  # Shape factor Cfx for longitudinal force
        self.p_dx1 = None  # Longitudinal friction Mux at Fznom
        self.p_dx3 = None  # Variation of friction Mux with camber
        self.p_ex1 = None  # Longitudinal curvature Efx at Fznom
        self.p_kx1 = None  # Longitudinal slip stiffness Kfx/Fz at Fznom
        self.p_hx1 = None  # Horizontal shift Shx at Fznom
        self.p_vx1 = None  # Vertical shift Svx/Fz at Fznom
        self.r_bx1 = None  # Slope factor for combined slip Fx reduction
        self.r_bx2 = None  # Variation of slope Fx reduction with kappa
        self.r_cx1 = None  # Shape factor for combined slip Fx reduction
        self.r_ex1 = None  # Curvature factor of combined Fx
        self.r_hx1 = None  # Shift factor for combined slip Fx reduction

        # lateral coefficients
        self.p_cy1 = None  # Shape factor Cfy for lateral forces
        self.p_dy1 = None  # Lateral friction Muy
        self.p_dy3 = None  # Variation of friction Muy with squared camber
        self.p_ey1 = None  # Lateral curvature Efy at Fznom
        self.p_ky1 = None  # Maximum value of stiffness Kfy/Fznom
        self.p_hy1 = None  # Horizontal shift Shy at Fznom
        self.p_hy3 = None  # Variation of shift Shy with camber
        self.p_vy1 = None  # Vertical shift in Svy/Fz at Fznom
        self.p_vy3 = None  # Variation of shift Svy/Fz with camber
        self.r_by1 = None  # Slope factor for combined Fy reduction
        self.r_by2 = None  # Variation of slope Fy reduction with alpha
        self.r_by3 = None  # Shift term for alpha in slope Fy reduction
        self.r_cy1 = None  # Shape factor for combined Fy reduction
        self.r_ey1 = None  # Curvature factor of combined Fy
        self.r_hy1 = None  # Shift factor for combined Fy reduction
        self.r_vy1 = None  # Kappa induced side force Svyk/Muy*Fz at Fznom
        self.r_vy3 = None  # Variation of Svyk/Muy*Fz with camber
        self.r_vy4 = None  # Variation of Svyk/Muy*Fz with alpha
        self.r_vy5 = None  # Variation of Svyk/Muy*Fz with kappa
        self.r_vy6 = None  # Variation of Svyk/Muy*Fz with atan(kappa)
