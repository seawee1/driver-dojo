import numpy as np

from driver_dojo.common import utils
from driver_dojo.observer import BaseObserver
from driver_dojo.common.utils import normalize_observations, create_observation_space
import driver_dojo.common.runtime_vars as runtime_vars


class EgoVehicleObserver(BaseObserver):
    """ Observe the ego vehicle state.
        Features:
            0: x-position
            1: y-position
            2: steering angle
            3: velocity
            4: acceleration
            5: angle
        """

    def __init__(self):
        super().__init__()
        x_low, y_low, x_high, y_high = (
            runtime_vars.net_bbox[0][0],
            runtime_vars.net_bbox[0][1],
            runtime_vars.net_bbox[1][0],
            runtime_vars.net_bbox[1][1],
        )

        self.low = np.array(
            [
                x_low,
                y_low,
                runtime_vars.vehicle.tum_params.steering.min,
                runtime_vars.vehicle.v_min,
                runtime_vars.vehicle.decel,
                -np.pi,
            ],
            dtype=np.float32,
        )
        self.high = np.array(
            [
                x_high,
                y_high,
                runtime_vars.vehicle.tum_params.steering.max,
                runtime_vars.vehicle.v_max,
                runtime_vars.vehicle.accel,
                np.pi,
            ],
            dtype=np.float32,
        )

        self.observation_space = create_observation_space(
            self.low,
            self.high,
            runtime_vars.config["observations"]["feature_scaling"],
        )

    def observe(self):
        obs = [
            runtime_vars.vehicle.position[0],
            runtime_vars.vehicle.position[1],
            runtime_vars.vehicle.steering_angle,
            runtime_vars.vehicle.speed,
            runtime_vars.vehicle.acceleration,
            utils.wrap_to_pi(runtime_vars.vehicle.angle),
        ]
        obs = normalize_observations(
            obs,
            self.low,
            self.high,
            runtime_vars.config["observations"]["feature_scaling"],
        )

        assert obs.shape == self.low.shape
        return obs
