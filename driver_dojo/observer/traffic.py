import numpy as np

from driver_dojo.observer import BaseObserver


class TrafficObserver(BaseObserver):
    """Observe other vehicle inside a certain radius around the ego vehicle. The observation vector contains:
    [x, y, rotation, vel, accel, decel, dist, [signals]]
    around the ego-vehicle and are sorted in ascending distance order.
    """

    def __init__(self, config, vehicle, traffic_manager):
        super().__init__(config, vehicle, traffic_manager)
        self._num_vehicles = self._obs_config.rvo_num_vehicles
        self._radius = self._obs_config.rvo_radius
        self._relative_obs = self._obs_config.relative_to_ego
        self._signals = self._obs_config.rvo_signals
        self._context_info = self._obs_config.rvo_context

        vel_low, vel_high = self._obs_config.rvo_speed_range
        accel_low, accel_high = self._obs_config.rvo_accel_range
        decel_low, decel_high = self._obs_config.rvo_accel_range
        if self._relative_obs:
            x_low, y_low, x_high, y_high = [-self._radius] * 2 + [self._radius] * 2
            vel_low = -vel_high
        else:
            x_low, y_low, x_high, y_high = -np.inf, -np.inf, np.inf, np.inf

        low = [x_low, y_low, -np.pi, vel_low, accel_low, decel_low, 0.0]
        high = [x_high, y_high, np.pi, vel_high, accel_high, decel_high, self._radius]

        if self._signals:
            low += [0, 0, 0]
            high += [1, 1, 1]
        if self._context_info:
            low += [0, 0, 0, 0]  # One-hot encoding for context types foe, follower, leader or neither-of-those
            high += [1, 1, 1, 1]

        self._num_feat = len(low)
        self.low = np.array(low * self._num_vehicles, dtype=np.float64)
        self.high = np.array(high * self._num_vehicles, dtype=np.float64)

    def step(self):
        ego_id = self.config.simulation.egoID
        traffic_state = self.traffic_manager.traffic_state
        ego_state = traffic_state[ego_id]
        traffic_state = traffic_state.mask(ego_id)

        if len(traffic_state) == 0:  # There is no traffic
            return self._normalize_obs(np.zeros_like(self.low, dtype=np.float64))

        dist = traffic_state.distances_to(ego_state)

        if self._relative_obs:
            traffic_state = traffic_state.relative_to(ego_state)

        radius_mask = np.argwhere(dist <= self._radius).flatten()
        if radius_mask.shape[0] == 0:  # No vehicle inside radius around ego
            return np.zeros_like(self.low, dtype=np.float64)
        dist = dist[radius_mask]  # Drop traffic outside radius
        sort_mask = np.argsort(dist).flatten()
        sort_mask = sort_mask[:self._num_vehicles]  # Drop if there are too many
        dist = dist[sort_mask]

        # Prepare obs vector
        num = len(dist)
        obs = np.zeros((num, self._num_feat))

        # Extract data from TrafficState, put into obs vec
        obs[:, [0, 1]] = traffic_state.location[radius_mask][sort_mask][:, [0, 1]]
        obs[:, 2] = traffic_state.rotation[radius_mask][sort_mask][:, 1]
        obs[:, 3] = traffic_state.velocity[radius_mask][sort_mask]
        obs[:, 4] = traffic_state.accel[radius_mask][sort_mask]
        obs[:, 5] = traffic_state.decel[radius_mask][sort_mask]
        obs[:, 6] = dist
        if self._signals:
            traffic_signals = traffic_state.signals[radius_mask][sort_mask]
            obs[:, 7] = [(int(x) & 0b0001) / 0b0001 for x in traffic_signals]
            obs[:, 8] = [(int(x) & 0b0010) / 0b0010 for x in traffic_signals]
            obs[:, 9] = [(int(x) & 0b1000) / 0b1000 for x in traffic_signals]
        if self._context_info:
            veh_ids = traffic_state.veh_id[radius_mask][sort_mask]
            ego_context = self.traffic_manager.get_ego_context()
            obs[:, 10] = [1 if x in ego_context['foes'] else 0 for x in veh_ids]
            obs[:, 11] = [1 if x in ego_context['followers'] else 0 for x in veh_ids]
            obs[:, 12] = [1 if x in ego_context['leaders'] else 0 for x in veh_ids]
            obs[:, 13] = ~(obs[:, 10].astype(bool) | obs[:, 11].astype(bool) | obs[:, 12].astype(bool))

        obs = obs.flatten()
        obs_padded = np.zeros_like(self.low, dtype=np.float64)  # Pad end with zeros
        obs_padded[:obs.shape[0]] = obs
        obs_padded = self._normalize_obs(obs_padded)  # Post-proc
        return obs_padded

    def explain(self):
        return ['x', 'y', 'rotation', 'velocity', 'acceleration', 'deceleration', 'distance', 'signal_0', 'signal_1', 'signal_2'] * self._num_vehicles

