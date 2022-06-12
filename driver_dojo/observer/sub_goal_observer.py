import numpy as np

from driver_dojo.observer.waypoint_observer import WaypointObserver
from driver_dojo.common.utils import create_observation_space
import driver_dojo.common.state_variables as state_variables


class SubGoalObserver(WaypointObserver):
    """Observe the next num_waypoints navigation returned by the WaypointManager.
    Features:
        0: x-coordinate
        1: y-coordinate
        2: distance to ego vehicle
        3: required rotation for ego vehicle to point towards it
    """

    def __init__(self):
        # This is just the constructor of the WaypointObserver with 'wp' config args changed to 'sub_goal'
        super().__init__()
        self.num_waypoints = state_variables.config["observations"]["sub_goal_num"]

        if state_variables.config["observations"]["sub_goal_xy_range"] is not None:
            x_low, y_low, x_high, y_high = state_variables.config["observations"][
                "sub_goal_xy_range"
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
        dist_high = state_variables.config["observations"]["sub_goal_dist_max"]

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

    def observe(self):
        sub_goals = state_variables.vehicle.sub_goals[
            : state_variables.config["observations"]["sub_goal_num"]
        ]
        obs = super().observe(sub_goals)
        return obs
