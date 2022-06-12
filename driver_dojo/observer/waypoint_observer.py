import numpy as np

from driver_dojo.common import utils
from driver_dojo.observer import BaseObserver
from driver_dojo.common.utils import normalize_observations, create_observation_space
import driver_dojo.common.state_variables as state_variables


class WaypointObserver(BaseObserver):
    """Observe the next num_waypoints navigation returned by the WaypointManager.
    Features:
        0: x-coordinate
        1: y-coordinate
        2: distance to ego vehicle
        3: required rotation for ego vehicle to point towards it
    """

    def __init__(self):
        super().__init__()
        self.num_waypoints = state_variables.config["observations"]["wp_num"]

        if state_variables.config["observations"]["wp_xy_range"] is not None:
            x_low, y_low, x_high, y_high = state_variables.config["observations"][
                "wp_xy_range"
            ]
        else:
            x_low, y_low, x_high, y_high = (
                state_variables.net_bbox[0][0],
                state_variables.net_bbox[0][1],
                state_variables.net_bbox[1][0],
                state_variables.net_bbox[1][1],
            )

        # Distance range
        dist_low = 0.0
        dist_high = state_variables.config["observations"]["wp_dist_max"]

        self.low = np.array(
            [x_low, y_low, dist_low, -np.pi] * self.num_waypoints, dtype=np.float32
        )
        self.high = np.array(
            [x_high, y_high, dist_high, np.pi] * self.num_waypoints, dtype=np.float32
        )

        self.observation_space = create_observation_space(
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )

    def observe(self, waypoints=None):
        waypoints = (
            state_variables.vehicle.waypoints if waypoints is None else waypoints
        )
        if len(waypoints) == 0:
            return np.zeros_like(self.low)

        xy_ego = state_variables.vehicle.position
        angle = utils.wrap_to_pi(state_variables.vehicle.angle)

        xy = np.array([waypoint.position for waypoint in waypoints])
        xy_rel = xy - xy_ego

        # Distance
        dists = np.linalg.norm(xy_rel, axis=1)
        dists = np.clip(dists, self.low[2], self.high[2])

        # Match angle
        angles = np.arctan2(xy_rel[:, 0], xy_rel[:, 1])
        match_angles = utils.wrap_to_pi(angles - angle)

        # Make xy-coordinates relative to ego position and rotation
        if state_variables.config["observations"]["relative_to_ego"]:
            theta = -angle + np.radians(90)
            rot = np.array(
                [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
            )
            xy = np.dot(rot, xy_rel.T).T

        # Clip into low high range
        # TODO: Actually, we have to project one circle around ego, but shouldn't be too important
        xy[:, 0] = np.clip(xy[:, 0], self.low[0], self.high[0])
        xy[:, 1] = np.clip(xy[:, 1], self.low[1], self.high[1])

        # Stack and flatten
        obs = np.hstack(
            [xy, np.expand_dims(dists, axis=1), np.expand_dims(match_angles, axis=1)]
        ).flatten()

        # Throws away waypoints we do not need
        obs = obs[: self.low.shape[0]]

        # Normalize
        obs = normalize_observations(
            obs,
            self.low[: obs.shape[0]],
            self.high[: obs.shape[0]],
            state_variables.config["observations"]["feature_scaling"],
        )

        # Appends 0 entries if not enough waypoints
        obs_ret = np.zeros_like(self.low)
        obs_ret[: obs.shape[0]] = obs

        assert obs_ret.shape == self.low.shape
        return obs_ret
