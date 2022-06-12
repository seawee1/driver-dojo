# from vehiclemodels.steeringParameters import SteeringParameters
# from vehiclemodels.utils.longitudinalParameters import LongitudinalParameters
from driver_dojo.vehicle.vehiclemodels.utils.longitudinal_parameters import LongitudinalParameters
from driver_dojo.vehicle.vehiclemodels.utils.steering_parameters import SteeringParameters
from driver_dojo.vehicle.vehiclemodels.utils.tireParameters import TireParameters
from driver_dojo.vehicle.vehiclemodels.utils.trailer_parameters import TrailerParameters


class VehicleParameters:
    def __init__(self):
        # vehicle body dimensions
        self.l = None
        self.w = None

        # steering parameters 
        self.steering = SteeringParameters()

        # longitudinal parameters 
        self.longitudinal = LongitudinalParameters()

        # masses
        self.m = None
        self.m_s = None
        self.m_uf = None
        self.m_ur = None

        # axes distances
        self.a = None  # distance from spring mass center of gravity to front axle [m]  LENA
        self.b = None  # distance from spring mass center of gravity to rear axle [m]  LENB

        # moments of inertia of sprung mass
        self.I_Phi_s = None  # moment of inertia for sprung mass in roll [kg m^2]  IXS
        self.I_y_s = None  # moment of inertia for sprung mass in pitch [kg m^2]  IYS
        self.I_z = None  # moment of inertia for sprung mass in yaw [kg m^2]  IZZ
        self.I_xz_s = None  # moment of inertia cross product [kg m^2]  IXZ

        # suspension parameters
        self.K_sf = None  # suspension spring rate (front) [N/m]  KSF
        self.K_sdf = None  # suspension damping rate (front) [N s/m]  KSDF
        self.K_sr = None  # suspension spring rate (rear) [N/m]  KSR
        self.K_sdr = None  # suspension damping rate (rear) [N s/m]  KSDR

        # geometric parameters
        self.T_f = None  # track width front [m]  TRWF
        self.T_r = None  # track width rear [m]  TRWB
        self.K_ras = None  # lateral spring rate at compliant compliant pin joint between M_s and M_u [N/m]  KRAS

        self.K_tsf = None  # auxiliary torsion roll stiffness per axle (normally negative) (front) [N m/rad]  KTSF
        self.K_tsr = None  # auxiliary torsion roll stiffness per axle (normally negative) (rear) [N m/rad]  KTSR
        self.K_rad = None  # damping rate at compliant compliant pin joint between M_s and M_u [N s/m]  KRADP
        self.K_zt = None  # vertical spring rate of tire [N/m]  TSPRINGR

        self.h_cg = None  # center of gravity height of total mass [m]  HCG (mainly required for conversion to other vehicle models)
        self.h_raf = None  # height of roll axis above ground (front) [m]  HRAF
        self.h_rar = None  # height of roll axis above ground (rear) [m]  HRAR

        self.h_s = None  # M_s center of gravity above ground [m]  HS

        self.I_uf = None  # moment of inertia for unsprung mass about x-axis (front) [kg m^2]  IXUF
        self.I_ur = None  # moment of inertia for unsprung mass about x-axis (rear) [kg m^2]  IXUR
        self.I_y_w = None  # wheel inertia, from internet forum for 235/65 R 17 [kg m^2]

        self.K_lt = None  # lateral compliance rate of tire, wheel, and suspension, per tire [m/N]  KLT
        self.R_w = None  # effective wheel/tire radius  chosen as tire rolling radius RR  taken from ADAMS documentation [m]

        # split of brake and engine torque
        self.T_sb = None
        self.T_se = None

        # suspension parameters
        self.D_f = None  # [rad/m]  DF
        self.D_r = None  # [rad/m]  DR
        self.E_f = None  # [needs conversion if nonzero]  EF
        self.E_r = None  # [needs conversion if nonzero]  ER

        # tire parameters 
        self.tire = TireParameters()

        # trailer parameters
        self.trailer = TrailerParameters()
