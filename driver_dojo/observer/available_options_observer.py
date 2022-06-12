import numpy as np

from driver_dojo.observer import BaseObserver
from driver_dojo.core.types import RoadOptionsExtended
from driver_dojo.common.utils import normalize_observations, create_observation_space
import driver_dojo.common.state_variables as state_variables


class AvailableOptionsObserver(BaseObserver):
    def __init__(self):
        super().__init__()
        self.low = np.array([0.0] * len(RoadOptionsExtended), dtype=np.float32)
        self.high = np.array([1.0] * len(RoadOptionsExtended), dtype=np.float32)
        self.observation_space = create_observation_space(
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )

    def observe(self):
        position = state_variables.vehicle.position
        laneID = state_variables.vehicle.laneID
        waypoint = state_variables.vehicle.waypoints[
            0
        ]  # state_variables.sumo_map.waypoint_on_lane(position, laneID)
        road_options = waypoint.road_options

        obs = np.zeros(self.low.shape)
        for road_option in road_options:
            obs[road_option] = 1.0

        obs = normalize_observations(
            obs,
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )

        assert obs.shape == self.low.shape

        return obs
