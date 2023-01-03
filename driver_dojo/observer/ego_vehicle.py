from abc import ABC

import numpy as np

from driver_dojo.common import utils
from driver_dojo.observer import BaseObserver
from driver_dojo.vehicle import TUMVehicle


class EgoVehicleObserver(BaseObserver):
    def __init__(self, config, vehicle, traffic_manager):
        super().__init__(config, vehicle, traffic_manager)
        self._vehicle = self.vehicle

        is_tum_vehicle = isinstance(self._vehicle, TUMVehicle)
        if is_tum_vehicle:
            steer_min = self._vehicle.dynamics_params.steering.min
            steer_max = self._vehicle.dynamics_params.steering.max
        else:  # Dummy values, we don't have steer information for other ego vehicle types
            steer_min = -1
            steer_max = 1
        veh_config = self.config.vehicle
        self.low = np.array(
            [
                -np.inf,
                -np.inf,
                steer_min,
                veh_config.v_min,
                veh_config.decel_max,
                -np.pi,
                -3.0
            ],
            dtype=np.float64,
        )
        self.high = np.array(
            [
                np.inf,
                np.inf,
                steer_max,
                veh_config.v_max,
                veh_config.accel_max,
                np.pi,
                3.0
            ],
            dtype=np.float64,
        )

    def step(self):
        veh_state = self._vehicle.state

        obs = [
            veh_state.location[0],
            veh_state.location[1],
            self._vehicle.steer,
            veh_state.velocity,
            veh_state.accel,
            utils.wrap_to_pi(veh_state.rotation)[1],
            veh_state.lane_position_lat
        ]

        return self._normalize_obs(obs)

    def explain(self):
        return ['x', 'y', 'steer', 'velocity', 'accel', 'yaw', 'lane_position_lat']
