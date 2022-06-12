import numpy as np

from driver_dojo.common import utils
from driver_dojo.observer import BaseObserver
from driver_dojo.common.utils import normalize_observations, create_observation_space
import driver_dojo.common.state_variables as state_variables


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
            state_variables.net_bbox[0][0],
            state_variables.net_bbox[0][1],
            state_variables.net_bbox[1][0],
            state_variables.net_bbox[1][1],
        )

        self.low = np.array(
            [
                x_low,
                y_low,
                state_variables.vehicle.tum_params.steering.min,
                state_variables.vehicle.v_min,
                state_variables.vehicle.decel,
                -np.pi,
            ],
            dtype=np.float32,
        )
        self.high = np.array(
            [
                x_high,
                y_high,
                state_variables.vehicle.tum_params.steering.max,
                state_variables.vehicle.v_max,
                state_variables.vehicle.accel,
                np.pi,
            ],
            dtype=np.float32,
        )

        self.observation_space = create_observation_space(
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )

    def observe(self):
        obs = [
            state_variables.vehicle.position[0],
            state_variables.vehicle.position[1],
            state_variables.vehicle.steering_angle,
            state_variables.vehicle.speed,
            state_variables.vehicle.acceleration,
            utils.wrap_to_pi(state_variables.vehicle.angle),
        ]
        obs = normalize_observations(
            obs,
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )

        assert obs.shape == self.low.shape
        return obs
