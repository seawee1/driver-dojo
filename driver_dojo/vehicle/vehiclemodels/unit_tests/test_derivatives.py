import unittest

from driver_dojo.vehicle.vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from driver_dojo.vehicle.vehiclemodels.parameters_vehicle4 import parameters_vehicle4
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_kst import vehicle_dynamics_kst
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std


class TestDerivatives(unittest.TestCase):
    def setUp(self):
        # load parameters
        self.p = parameters_vehicle2()
        g = 9.81
        # set input
        v_delta = 0.15
        acc = 0.63 * g
        self.u = [v_delta, acc]

    def test_ks(self):
        # state values
        x_ks = [3.9579422297936526, 0.0391650102771405, 0.0378491427211811, 16.3546957860883566, 0.0294717351052816]
        # ground truth
        f_ks_gt = [16.3475935934250209, 0.4819314886013121, 0.1500000000000000, 5.1464424102339752, 0.2401426578627629]
        # test
        f_ks = vehicle_dynamics_ks(x_ks, self.u, self.p)
        for i in range((len(f_ks))):
            self.assertAlmostEqual(f_ks[i], f_ks_gt[i], places=14)

    def test_kst(self):
        # state values
        x_kst = [3.9579422297936526, 0.0391650102771405, 0.0378491427211811, 16.3546957860883566, 0.0294717351052816, 0.0294717351052816]
        # ground truth
        f_kst_gt = [16.3475935934250209, 0.4819314886013121, 0.1500000000000000, 5.501539201758522, 0.1720297150523055, -0.23152742969444276]
        # test
        f_kst = vehicle_dynamics_kst(x_kst, self.u, parameters_vehicle4())
        for i in range(len(f_kst)):
            self.assertAlmostEqual(f_kst[i], f_kst_gt[i], places=14)

    def test_st(self):
        # state values
        x_st = [2.0233348142065677, 0.0041907137716636, 0.0197545248559617, 15.7216236334290116, 0.0025857914776859,
                0.0529001056654038, 0.0033012170610298]
        # ground truth
        f_st_gt = [15.7213512030862397, 0.0925527979719355, 0.1500000000000000, 5.3536773276413925, 0.0529001056654038,
                   0.6435589397748606, 0.0313297971641291]
        # test
        f_st = vehicle_dynamics_st(x_st, self.u, self.p)
        for i in range((len(f_st))):
            self.assertAlmostEqual(f_st[i], f_st_gt[i], places=14)

    def test_std(self):
        # state values
        x_std = [2.0233348142065677, 0.0041907137716636, 0.0197545248559617, 15.7216236334290116, 0.0025857914776859,
                0.0529001056654038, 0.0033012170610298, 53.6551710641082451, 56.7917911784219598]
        # ground truth
        f_std_gt = [15.72135120308624, 0.09255279797193551, 0.15, 11.359186518121511, 0.0529001056654038, 0.5870894694428362,
                    -0.007910389538735838, -1403.4833546513996, 72.25313872307531]
        # test
        f_std = vehicle_dynamics_std(x_std, self.u, self.p)
        for i in range(len(f_std)):
            self.assertAlmostEqual(f_std[i], f_std_gt[i], places=14)

    def test_mb(self):
        # state values
        x_mb = [10.8808433066274794, 0.5371850187869442, 0.0980442671005920, 18.0711398687457745, 0.1649995631003776,
                0.6158755000936103, -0.1198403612262477, -0.2762672756169581, 0.0131909920269115, -0.0718483683742141,
                -0.3428324595054725, 0.0103233373083297, -0.0399590564140291, -0.0246468320579360, -0.0551575051990853,
                0.5798277643297529, 0.0172059354801703, 0.0067890113155477, -0.0184269459410162, -0.0408207136116175,
                -1.0064484829203018, 0.0166808347900582, -0.0049188492004049, 53.6551710641082451, 50.7045242506744316,
                56.7917911784219598, 200.7079633169796296, -0.0939969123691911, -0.0881514621614376]
        # ground truth
        f_mb_gt = [17.8820162482414098, 2.6300428035858809, 0.1500000000000000, 2.7009396644636450, 0.6158755000936103,
                   1.3132879472301846, -0.2762672756169581, -0.1360581472638375, -0.0718483683742141,
                   0.4909514227532223, -2.6454134031927374, -0.0399590564140291, 0.0486778649724968,
                   -0.0551575051990853, 0.0354501802049087, -1.1130397141873534, 0.0067890113155477,
                   -0.0886810139593130, -0.0408207136116175, 0.0427680029698811, -4.3436374104751501,
                   -0.0049188492004049, 0.1142109377736169, 10.0527321757776047, 0.0436512393438736,
                   -154.12795711273924, 422.12780928288305, -0.2105876149500405, -0.2126005780977984]
        # test
        f_mb = vehicle_dynamics_mb(x_mb, self.u, self.p)
        for i in range((len(f_mb))):
            self.assertAlmostEqual(f_mb[i], f_mb_gt[i])


if __name__ == '__main__':
    unittest.main()
