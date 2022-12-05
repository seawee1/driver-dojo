import numpy as np

from driver_dojo.common import utils
from driver_dojo.observer import BaseObserver


class WaypointObserver(BaseObserver):
    """Observe the next num_waypoints navigation returned by the WaypointManager.
    Features:
        0: x-coordinate
        1: y-coordinate
        2: distance to ego vehicle
        3: required rotation for ego vehicle to point towards it
    """

    def __init__(self, config, vehicle, traffic_manager):
        super().__init__(config, vehicle, traffic_manager)
        self._num_wps = self.config.observations.wp_num
        self._sampling_dist = self.config.observations.wp_sampling_dist
        if self.config.observations.relative_to_ego:
            x_high = self._num_wps * self._sampling_dist
            x_low = -x_high
            y_low = 0.0
            y_high = x_high
        else:
            x_low, y_low, x_high, y_high = -np.inf, -np.inf, np.inf, np.inf

        dist_low = 0.0
        dist_high = self._num_wps * self._sampling_dist

        self.low = np.array([x_low, y_low, dist_low, -np.pi] * self._num_wps, dtype=np.float64)
        self.high = np.array([x_high, y_high, dist_high, np.pi] * self._num_wps, dtype=np.float64)

    def step(self, xs=None, ys=None):
        if xs is None and ys is None:  # This is always the case for WaypointObserver. For SubGoalObserver, we pass those
            xs, ys, _, _, _ = self.vehicle.waypoint_attachment.cubic_spline_course(self.vehicle.location[:2])  # Get the course
            course_res = self.config.vehicle.course_resolution
            idx_step = int(self._sampling_dist / course_res)
            waypoints_idx = [(i+1)*idx_step for i in range(self._num_wps) if (i+1)*idx_step < len(xs)]  # Sample waypoints and their coordinates
            xs, ys = [xs[i] for i in waypoints_idx], [ys[i] for i in waypoints_idx]

        xy = np.hstack([np.expand_dims(np.array(xs), axis=1), np.expand_dims(np.array(ys), axis=1)])

        xy_ego = self.vehicle.location[:2]  # Get ego transform
        angle = utils.wrap_to_pi(self.vehicle.rotation[1])

        xy_rel = xy - xy_ego

        dists = np.linalg.norm(xy_rel, axis=1)  # Distances
        dists = np.clip(dists, self.low[2], self.high[2])

        angles = utils.wrap_to_pi(np.arctan2(xy_rel[:, 1], xy_rel[:, 0]))  # Match angles
        match_angles = utils.wrap_to_pi(angles - angle)

        if self.config.observations.relative_to_ego:
            theta = -angle + np.radians(90)
            rot = np.array(
                [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
            )
            xy = np.dot(rot, xy_rel.T).T

        obs = np.hstack(
            [xy, np.expand_dims(dists, axis=1), np.expand_dims(match_angles, axis=1)]
        ).flatten()

        obs_ret = np.zeros_like(self.low)
        obs_ret[:obs.shape[0]] = obs
        obs_ret = self._normalize_obs(obs_ret)
        obs_ret[obs.shape[0]:] = 0.0  # Make dummy entries zero again
        return obs_ret

    def explain(self):
        return ['x', 'y', 'dist', 'match_angle'] * self._num_wps
