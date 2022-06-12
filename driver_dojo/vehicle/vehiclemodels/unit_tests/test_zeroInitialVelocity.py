from driver_dojo.vehicle.vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from driver_dojo.vehicle.vehiclemodels.init_ks import init_ks
from driver_dojo.vehicle.vehiclemodels.init_st import init_st
from driver_dojo.vehicle.vehiclemodels.init_mb import init_mb
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb
from scipy.integrate import odeint
import numpy


def func_KS(x, t, u, p):
    f = vehicle_dynamics_ks(x, u, p)
    return f
    

def func_ST(x, t, u, p):
    f = vehicle_dynamics_st(x, u, p)
    return f


def func_MB(x, t, u, p):
    f = vehicle_dynamics_mb(x, u, p)
    return f


def test_zeroInitialVelocity():
  # test_zeroInitialVelocity - unit_test_function for starting with zero 
  # initial velocity
  #
  # Some vehicle models have a singularity when the vehicle is not moving.
  # This test checks whether the vehicle can handle this situation
  #
  # Syntax:  
  #    res = test_zeroInitialVelocity()
  #
  # Inputs:
  #    ---
  #
  # Outputs:
  #    res - boolean result (0/empty: test not passed, 1: test passed)
  #
  # Example: 
  #
  # Other m-files required: none
  # Subfunctions: none
  # MAT-files required: none
  #
  # See also: ---

  # Author:       Matthias Althoff
  # Written:      16-December-2017
  # Last update:  ---
  # Last revision:---

  #------------- BEGIN CODE --------------

  # initialize result
  res = []

  # load parameters
  p = parameters_vehicle2()
  g = 9.81  #[m/s^2]

  # set options --------------------------------------------------------------
  tStart = 0  #start time
  tFinal = 1  #start time

  delta0 = 0
  vel0 = 0
  Psi0 = 0
  dotPsi0 = 0
  beta0 = 0
  sy0 = 0
  initialState = [0,sy0,delta0,vel0,Psi0,dotPsi0,beta0]  #initial state for simulation
  x0_KS = init_ks(initialState)  #initial state for kinematic single-track model
  x0_ST = init_st(initialState)  #initial state for single-track model
  x0_MB = init_mb(initialState, p)  #initial state for multi-body model
  #--------------------------------------------------------------------------

  # create time vector
  t = numpy.arange(tStart,tFinal, 1e-4)
  
  # set input: rolling car (velocity should stay constant)
  u = [0, 0]

  # simulate multi-body model
  x_roll = odeint(func_MB, x0_MB, t, args=(u, p))

  # simulate single-track model
  x_roll_st = odeint(func_ST, x0_ST, t, args=(u, p))

  # simulate kinematic single-track model
  x_roll_ks = odeint(func_KS, x0_KS, t, args=(u, p))

  # check correctness
  # ground truth
  x_roll_gt = [0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, -0.0000000003174207, 0.0000000848065981, -0.0013834133396573, -0.0020336367252011, -0.0000000247286655, 0.0176248518072475, 0.0071655470428753, 0.0000000006677358, -0.0000001709775865, 0.0000001839820148, 0.0186763737562366, 0.0003752526345970, 0.0000000006728055, -0.0000001734436431, 0.0000001850020879, 0.0154621865353889, 0.0000251622262094, -0.0000174466440656, -0.0000174466440656, -0.0000014178345014, -0.0000014178345014, 0.0000000008088692, 0.0000000008250785]
  
  #comparison
  res.append(all(abs(x_roll[-1] - x_roll_gt) < 1e-2)) # -1 refers to last time step
  res.append(all(x_roll_st[-1] == x0_ST)) # -1 refers to last time step
  res.append(all(x_roll_ks[-1] == x0_KS)) # -1 refers to last time step
  print(res)
  print(x_roll[0])
  print(x_roll[-1])
  print(abs(x_roll[0] - x_roll_gt))
  print(abs(x_roll[-1] - x_roll_gt))
  #--------------------------------------------------------------------------

  # set input: decelerating car ---------------------------------------------
  v_delta = 0
  acc = -0.7*g
  u = [v_delta, acc]

  # simulate multi-body model
  x_dec = odeint(func_MB,x0_MB, t, args=(u, p))

  # simulate single-track model
  x_dec_st = odeint(func_ST,x0_ST, t, args=(u, p))

  # simulate kinematic single-track model
  x_dec_ks = odeint(func_KS,x0_KS, t,  args=(u, p), rtol=1e-14)

  # check correctness
  #ground truth
  x_dec_gt = [3.9830932439714273, -0.0601543816394762, 0.0000000000000000, -8.0013986587693893, -0.0026467910011602, -0.0053025639381130, -0.0019453336082831, -0.0002270008481486, -0.0431740570135473, -0.0305313864800163, 0.1053033709671264, 0.0185102262795369, 0.0137681838589757, -0.0003400843778018, -0.0000161129034355, 0.0994502177784092, 0.0256268504637763, 0.0034700280714177, -0.0002562443897593, -0.0000034699487925, 0.1128675292571426, 0.0086968977905411, -0.0020987862166353, -0.0000183158385631, -0.0000183158385631, -0.0000095073736467, -0.0000095073736467, -0.0016872664171374, -0.0012652511246015]  # ground truth for multi-body model
  x_dec_st_gt = [-3.4335000000000013, 0.0000000000000000, 0.0000000000000000, -6.8670000000000018, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000]  # ground truth for single-track model
  x_dec_ks_gt = [-3.4335000000000013, 0.0000000000000000, 0.0000000000000000, -6.8670000000000018, 0.0000000000000000]  # ground truth for kinematic single-track model
  
  #compare
  res.append(all(abs(x_dec[-1] - x_dec_gt) < 1e-2)) # -1 refers to last time step
  res.append(all(abs(x_dec_st[-1] - x_dec_st_gt) < 1e-2)) # -1 refers to last time step
  res.append(all(abs(x_dec_ks[-1] - x_dec_ks_gt) < 1e-2)) # -1 refers to last time step
  print(res)
  print(abs(x_dec[-1] - x_dec_gt))
  print('KS check:')
  print(abs(x_dec_ks[-1] - x_dec_ks_gt))
  #--------------------------------------------------------------------------


  # set input: accelerating car (wheel spin and velocity should increase more wheel spin at rear)
  v_delta = 0.15
  acc = 0.63*g
  u = [v_delta, acc]

  # simulate multi-body model
  x_acc = odeint(func_MB, x0_MB, t, args=(u, p))

  # simulate single-track model
  x_acc_st = odeint(func_ST, x0_ST, t, args=(u, p))

  # simulate kinematic single-track model
  x_acc_ks = odeint(func_KS, x0_KS, t, args=(u, p))

  # check correctness
  #ground truth
  x_acc_gt = [1.6869441956852231, 0.0041579276718349, 0.1500000000000001, 3.1967387404602654, 0.3387575860582390, 0.8921302762726965, -0.0186007698209413, -0.0556855538608812, 0.0141668816602887, 0.0108112584162600, -0.6302339461329982, 0.0172692751292486, 0.0025948291288222, -0.0042209020256358, -0.0115749221900647, 0.4525764527765288, 0.0161366380049974, -0.0012354790918115, -0.0023647389844973, -0.0072210348979615, -1.8660984955372673, 0.0179591511062951, 0.0010254111038481, 11.1322413877606117, 7.5792605585643713, 308.3079237740076906, 310.8801727728298374, -0.0196922024889714, -0.0083685253175425]
  x_acc_st_gt = [3.0731976046859715, 0.2869835398304389, 0.1500000000000000, 6.1802999999999999, 0.1097747074946325, 0.3248268063223301, 0.0697547542798040]  # ground truth for single-track model
  x_acc_ks_gt = [3.0845676868494927, 0.1484249221523042, 0.1500000000000000, 6.1803000000000017, 0.1203664469224163]  # ground truth for kinematic single-track model
    
  #compare
  res.append(all(abs(x_acc[-1] - x_acc_gt) < 1e-2)) # -1 refers to last time step
  res.append(all(abs(x_acc_st[-1] - x_acc_st_gt) < 1e-2)) # -1 refers to last time step
  res.append(all(abs(x_acc_ks[-1] - x_acc_ks_gt) < 1e-2)) # -1 refers to last time step
  print(res)
  print(abs(x_acc[-1] - x_acc_gt))
  print(abs(x_acc_ks[-1] - x_acc_ks_gt))
  #--------------------------------------------------------------------------


  # steering to left---------------------------------------------------------
  v_delta = 0.15
  u = [v_delta, 0]

  # simulate multi-body model
  x_left = odeint(func_MB, x0_MB, t, args=(u, p))

  # simulate single-track model
  x_left_st = odeint(func_ST, x0_ST, t, args=(u, p))

  # simulate kinematic single-track model
  x_left_ks = odeint(func_KS, x0_KS, t, args=(u, p))

  # check correctness
  x_left_gt = [0.0000000000000000, 0.0000000000000000, 0.1500000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0003021160057306, 0.0115474648881108, -0.0013797955031689, -0.0019233204598741, -0.0065044050021887, 0.0176248291065725, 0.0071641239008779, 0.0001478513434683, 0.0092020911982902, -0.0178028732533553, 0.0186751057310096, 0.0003566948613572, 0.0001674970785214, 0.0015871955172538, -0.0175512251679294, 0.0154636630992985, 0.0000482191918813, -0.0000173442953338, -0.0000174708138706, -0.0000014178345014, -0.0000014178345014, 0.0002293337149155, 0.0003694012334077]
  x_left_st_gt = [0.0000000000000000, 0.0000000000000000, 0.1500000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000]  # ground truth for single-track model
  x_left_ks_gt = [0.0000000000000000, 0.0000000000000000, 0.1500000000000000, 0.0000000000000000, 0.0000000000000000]  # ground truth for kinematic single-track model
  
    #compare
  res.append(all(abs(x_left[-1] - x_left_gt) < 1e-2)) # -1 refers to last time step
  res.append(all(abs(x_left_st[-1] - x_left_st_gt) < 1e-2)) # -1 refers to last time step
  res.append(all(abs(x_left_ks[-1] - x_left_ks_gt) < 1e-2)) # -1 refers to last time step
  print(res)
  print(x_left[-1])
  print(x_left[0])
  print(x_left[10])
  print(x_left[100])
  print(abs(x_left[-1] - x_left_gt))
  print(abs(x_left_ks[-1] - x_left_ks_gt))
  #--------------------------------------------------------------------------

  # obtain final result
  res = all(res)
  print(res)
  
  return res

  #------------- END OF CODE --------------
