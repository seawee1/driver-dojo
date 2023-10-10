import numpy as np

from driver_dojo.observer import BaseObserver
from driver_dojo.core.types import RoadOptionsExtended
from driver_dojo.vehicle.attachments import WaypointAttachment


class AvailableOptionsObserver(BaseObserver):
    def __init__(self, config, vehicle, traffic_manager):
        super().__init__(config, vehicle, traffic_manager)
        self.low = np.array([0.0] * len(RoadOptionsExtended), dtype=np.float64)
        self.high = np.array([1.0] * len(RoadOptionsExtended), dtype=np.float64)

    def step(self):
        assert np.any([isinstance(x, WaypointAttachment) for x in self.vehicle.attachments])
        wp = self.vehicle.waypoints[0]
        road_opt = wp.road_options

        obs = np.zeros(self.low.shape)
        for road_option in road_opt:
            obs[road_option] = 1.0

        return self._normalize_obs(obs)

    def explain(self):
        return ['Switch-left', 'Switch-right', 'Follow', 'Left', 'Right']
