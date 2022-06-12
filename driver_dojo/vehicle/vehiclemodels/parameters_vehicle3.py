from driver_dojo.vehicle.vehiclemodels.utils import unitConversion
from driver_dojo.vehicle.vehiclemodels.vehicle_parameters import VehicleParameters


def parameters_vehicle3():
    # parameters_vehicle3 - parameter set of the multi-body vehicle dynamics 
    # based on the DOT (department of transportation) vehicle dynamics 
    # values are taken from a VW Vanagon
    #
    # Syntax:  
    #    p = parameters_vehicle3()
    #
    # Inputs:
    #    ---
    #
    # Outputs:
    #    p - parameter vector
    #
    # Example: 
    #
    # Other m-files required: none
    # Subfunctions: none
    # MAT-files required: none
    #
    # See also: ---

    # Author:       Matthias Althoff
    # Written:      15-January-2017
    # Last update:05-July-2017
    #                     16-December-2017
    # Last revision:---

    #------------- BEGIN CODE --------------

    #init vehicle parameters
    p = VehicleParameters()
    
    #vehicle body dimensions
    p.l = 4.569  #vehicle length [m] 
    p.w = 1.844  #vehicle width [m]
    
    #steering constraints
    p.steering.min = -1.023  #minimum steering angle [rad]
    p.steering.max = 1.023  #maximum steering angle [rad]
    p.steering.v_min = -0.4  #minimum steering velocity [rad/s]
    p.steering.v_max = 0.4  #maximum steering velocity [rad/s]

    #longitudinal constraints
    p.longitudinal.v_min = -11.2  #minimum velocity [m/s]
    p.longitudinal.v_max = 41.7  #minimum velocity [m/s]
    p.longitudinal.v_switch = 7.824  #switching velocity [m/s]
    p.longitudinal.a_max = 11.5  #maximum absolute acceleration [m/s^2]

    #masses
    p.m = unitConversion.lb_sec2_ft_IN_kg(101.3367)  #vehicle mass [kg]  MASS
    p.m_s = unitConversion.lb_sec2_ft_IN_kg(90.21635)  #sprung mass [kg]  SMASS
    p.m_uf = unitConversion.lb_sec2_ft_IN_kg(5.56015)  #unsprung mass front [kg]  UMASSF
    p.m_ur = unitConversion.lb_sec2_ft_IN_kg(5.56015)  #unsprung mass rear [kg]  UMASSR

    #axes distances
    p.a = unitConversion.ft_IN_m(3.775563)  #distance from spring mass center of gravity to front axle [m]  LENA
    p.b = unitConversion.ft_IN_m(4.334437)  #distance from spring mass center of gravity to rear axle [m]  LENB

    #moments of inertia of sprung mass
    p.I_Phi_s = unitConversion.lb_ft_sec2_IN_kg_m2(353.9445)  #moment of inertia for sprung mass in roll [kg m^2]  IXS
    p.I_y_s = unitConversion.lb_ft_sec2_IN_kg_m2(1625.825)  #moment of inertia for sprung mass in pitch [kg m^2]  IYS
    p.I_z = unitConversion.lb_ft_sec2_IN_kg_m2(1824.078)  #moment of inertia for sprung mass in yaw [kg m^2]  IZZ
    p.I_xz_s = unitConversion.lb_ft_sec2_IN_kg_m2(0)  #moment of inertia cross product [kg m^2]  IXZ

    #suspension parameters
    p.K_sf = unitConversion.lbs_ft_IN_N_m(2300)  #suspension spring rate (front) [N/m]  KSF
    p.K_sdf = unitConversion.lb_sec_ft_IN_N_s_m(164.8335)  #suspension damping rate (front) [N s/m]  KSDF
    p.K_sr = unitConversion.lbs_ft_IN_N_m(2680)  #suspension spring rate (rear) [N/m]  KSR
    p.K_sdr = unitConversion.lb_sec_ft_IN_N_s_m(189.7866)  #suspension damping rate (rear) [N s/m]  KSDR

    #geometric parameters
    p.T_f = unitConversion.ft_IN_m(5.165)  #track width front [m]  TRWF
    p.T_r = unitConversion.ft_IN_m(5.065)  #track width rear [m]  TRWB
    p.K_ras = unitConversion.lbs_ft_IN_N_m(12000)  #lateral spring rate at compliant compliant pin joint between M_s and M_u [N/m]  KRAS

    p.K_tsf = unitConversion.ft_lb_rad_IN_N_m_rad(-25038.92)  #auxiliary torsion roll stiffness per axle (normally negative) (front) [N m/rad]  KTSF
    p.K_tsr = unitConversion.ft_lb_rad_IN_N_m_rad(-5702.369)  #auxiliary torsion roll stiffness per axle (normally negative) (rear) [N m/rad]  KTSR
    p.K_rad = unitConversion.lb_sec_ft_IN_N_s_m(700)  # damping rate at compliant compliant pin joint between M_s and M_u [N s/m]  KRADP
    p.K_zt = unitConversion.lbs_ft_IN_N_m(14565.6)  # vertical spring rate of tire [N/m]  TSPRINGR

    p.h_cg = unitConversion.ft_IN_m(2.453467)  #center of gravity height of total mass [m]  HCG (mainly required for conversion to other vehicle models)
    p.h_raf = unitConversion.ft_IN_m(0)  #height of roll axis above ground (front) [m]  HRAF
    p.h_rar = unitConversion.ft_IN_m(0)  #height of roll axis above ground (rear) [m]  HRAR

    p.h_s = unitConversion.ft_IN_m(2.639405)  #M_s center of gravity above ground [m]  HS

    p.I_uf = unitConversion.lb_ft_sec2_IN_kg_m2(37.08234)  #moment of inertia for unsprung mass about x-axis (front) [kg m^2]  IXUF
    p.I_ur = unitConversion.lb_ft_sec2_IN_kg_m2(35.66033)  #moment of inertia for unsprung mass about x-axis (rear) [kg m^2]  IXUR
    p.I_y_w = 1.7  #wheel inertia, from internet forum for 235/65 R 17 [kg m^2]

    p.K_lt = unitConversion.ft_lb_IN_m_N(1.7850283e-04)  #lateral compliance rate of tire, wheel, and suspension, per tire [m/N]  KLT
    p.R_w = 0.344  #effective wheel/tire radius  chosen as tire rolling radius RR  taken from ADAMS documentation [m]

    #split of brake and engine torque
    p.T_sb = 0.64 
    p.T_se = 0.0

    #suspension parameters
    p.D_f = unitConversion.rad_ft_IN_rad_m(-0.1)  #[rad/m]  DF; modified from orig. paper due to stability issues when damping is ignored
    p.D_r = unitConversion.rad_ft_IN_rad_m(-0.1)  #[rad/m]  DR; modified from orig. paper due to stability issues when damping is ignored
    p.E_f = 0.0  #[needs conversion if nonzero]  EF
    p.E_r = 0.0  #[needs conversion if nonzero]  ER


    #tire parameters from ADAMS handbook
    #longitudinal coefficients
    p.tire.p_cx1 = 1.6411  #Shape factor Cfx for longitudinal force
    p.tire.p_dx1 = 1.1739  #Longitudinal friction Mux at Fznom
    p.tire.p_dx3 = 0.0  #Variation of friction Mux with camber
    p.tire.p_ex1 = 0.46403  #Longitudinal curvature Efx at Fznom
    p.tire.p_kx1 = 22.303  #Longitudinal slip stiffness Kfx/Fz at Fznom
    p.tire.p_hx1 = 0.0012297  #Horizontal shift Shx at Fznom
    p.tire.p_vx1 = -8.8098e-006  #Vertical shift Svx/Fz at Fznom
    p.tire.r_bx1 = 13.276  #Slope factor for combined slip Fx reduction
    p.tire.r_bx2 = -13.778  #Variation of slope Fx reduction with kappa
    p.tire.r_cx1 = 1.2568  #Shape factor for combined slip Fx reduction
    p.tire.r_ex1 = 0.65225  #Curvature factor of combined Fx
    p.tire.r_hx1 = 0.0050722  #Shift factor for combined slip Fx reduction

    #lateral coefficients
    p.tire.p_cy1 = 1.3507  #Shape factor Cfy for lateral forces
    p.tire.p_dy1 = 1.0489  #Lateral friction Muy
    p.tire.p_dy3 = -2.8821  #Variation of friction Muy with squared camber
    p.tire.p_ey1 = -0.0074722  #Lateral curvature Efy at Fznom
    p.tire.p_ky1 = -21.92  #Maximum value of stiffness Kfy/Fznom
    p.tire.p_hy1 = 0.0026747  #Horizontal shift Shy at Fznom
    p.tire.p_hy3 = 0.031415  #Variation of shift Shy with camber
    p.tire.p_vy1 = 0.037318  #Vertical shift in Svy/Fz at Fznom
    p.tire.p_vy3 = -0.32931  #Variation of shift Svy/Fz with camber
    p.tire.r_by1 = 7.1433  #Slope factor for combined Fy reduction
    p.tire.r_by2 = 9.1916  #Variation of slope Fy reduction with alpha
    p.tire.r_by3 = -0.027856  #Shift term for alpha in slope Fy reduction
    p.tire.r_cy1 = 1.0719  #Shape factor for combined Fy reduction
    p.tire.r_ey1 = -0.27572  #Curvature factor of combined Fy
    p.tire.r_hy1 = 5.7448e-006  #Shift factor for combined Fy reduction
    p.tire.r_vy1 = -0.027825  #Kappa induced side force Svyk/Muy*Fz at Fznom
    p.tire.r_vy3 = -0.27568  #Variation of Svyk/Muy*Fz with camber
    p.tire.r_vy4 = 12.12  #Variation of Svyk/Muy*Fz with alpha
    p.tire.r_vy5 = 1.9  #Variation of Svyk/Muy*Fz with kappa
    p.tire.r_vy6 = -10.704  #Variation of Svyk/Muy*Fz with atan(kappa)

    return p

#------------- END OF CODE --------------
