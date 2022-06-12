import numpy as np

from driver_dojo.observer import BaseObserver
from driver_dojo.common.utils import normalize_observations, create_observation_space
import driver_dojo.common.state_variables as state_variables


class RadiusVehicleObserver(BaseObserver):
    """Observe other vehicle inside a certain radius around the ego vehicle. The observation vector contains
    [x_0, y_0, distance_0, vel_0, accel_0 yaw_0 (rad), ..., x_n, y_n, ...] of traffic participants inside a pre-defined radius
    around the ego-vehicle and are sorted in ascending distance order.
    """

    def __init__(self):
        super().__init__()
        self.num_vehicles = state_variables.config["observations"]["rvo_num_vehicles"]
        self.radius = state_variables.config["observations"]["rvo_radius"]
        self.relative = state_variables.config["observations"]["relative_to_ego"]
        self.signals = state_variables.config["observations"]["rvo_signals"]

        if self.relative:
            x_low, y_low, x_high, y_high = (
                -self.radius,
                -self.radius,
                self.radius,
                self.radius,
            )
        else:
            x_low, y_low, x_high, y_high = state_variables.net_bbox

        speed_low, speed_high = state_variables.config["observations"][
            "rvo_speed_range"
        ]
        acceleration_low, acceleration_high = state_variables.config["observations"][
            "rvo_accel_range"
        ]

        if self.relative:
            speed_low = -speed_high

        low = [x_low, y_low, 0.0, speed_low, acceleration_low, -np.pi]
        high = [x_high, y_high, self.radius, speed_high, acceleration_high, np.pi]
        if self.signals:
            low += [0, 0, 0]
            high += [1, 1, 1]
        self.low = np.array(low * self.num_vehicles, dtype=np.float32)
        self.high = np.array(high * self.num_vehicles, dtype=np.float32)

        self.observation_space = create_observation_space(
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )

    def observe(self):
        dists = state_variables.traffic_manager.distance_to_ego
        if dists is None:
            return np.zeros_like(self.low, dtype=np.float32)

        mask_radius = np.argwhere(dists <= self.radius).flatten()
        if mask_radius.shape[0] == 0:
            return np.zeros_like(self.low, dtype=np.float32)

        if self.relative:
            traffic_state = (
                state_variables.traffic_manager.traffic_state_transformed_relative()
            )
        else:
            traffic_state = state_variables.traffic_manager.traffic_state_transformed()

        # Drop traffic outside radius
        traffic_state = traffic_state[mask_radius]
        dists = dists[mask_radius]

        # Sort based on distance
        mask_sort = np.argsort(dists).flatten()
        traffic_state = traffic_state[mask_sort]
        dists = dists[mask_sort]

        # Drop, if there are too many vehicle to observe
        traffic_state = traffic_state[: self.num_vehicles]
        dists = dists[: self.num_vehicles]

        # Create observation vector
        num = traffic_state.shape[0]
        obs_data = np.zeros((num, 9)) if self.signals else np.zeros((num, 6))
        var_to_index = state_variables.traffic_manager.index
        obs_data[:, [0, 1, 3, 4, 5]] = traffic_state[
            :,
            [
                var_to_index["position"][0],
                var_to_index["position"][1],
                var_to_index["speed"],
                var_to_index["acceleration"],
                var_to_index["angle"],
            ],
        ]
        if self.signals:
            # Signals
            # VEH_SIGNAL_BLINKER_RIGHT: bit 0
            # VEH_SIGNAL_BLINKER_LEFT: bit 1
            # VEH_SIGNAL_BRAKELIGHT: bit 3
            signals = traffic_state[:, state_variables.traffic_manager.index["signals"]]
            obs_data[:, 6] = [(int(x) & 0b0001) / 0b0001 for x in signals]
            obs_data[:, 7] = [(int(x) & 0b0010) / 0b0010 for x in signals]
            obs_data[:, 8] = [(int(x) & 0b1000) / 0b1000 for x in signals]
        obs_data[:, 2] = dists
        obs_data = obs_data.flatten()

        # Perform norm/standardization
        obs_data = normalize_observations(
            obs_data,
            self.low[: obs_data.shape[0]],
            self.high[: obs_data.shape[0]],
            state_variables.config["observations"]["feature_scaling"],
        )

        # Fill end with zeros if necessary
        obs_ret = np.zeros(self.low.shape[0], dtype=np.float32)
        obs_ret[: obs_data.shape[0]] = obs_data

        assert obs_ret.shape == self.low.shape
        return obs_ret
