import numpy as np

from driver_dojo.observer import WaypointObserver
from driver_dojo.vehicle.attachments import SubGoalAttachment


class SubGoalObserver(WaypointObserver):
    def __init__(self, config, vehicle, traffic_manager):
        super().__init__(config, vehicle, traffic_manager)
        # This is just the constructor of the WaypointObserver with 'wp' config args changed to 'sub_goal'
        self._num_wps = self.config.observations.sub_goal_num

        sub_goal_dist_max = self.config.observations.sub_goal_dist_max
        if sub_goal_dist_max is not None and self.config.observations.relative_to_ego:
            x_low, y_low, x_high, y_high = -sub_goal_dist_max, -sub_goal_dist_max, sub_goal_dist_max, sub_goal_dist_max
        else:
            x_low, y_low, x_high, y_high = -np.inf, -np.inf, np.inf, np.inf

        dist_low = 0.0
        dist_high = self.config.observations.sub_goal_dist_max

        self.low = np.array(
            [x_low, y_low, dist_low, -np.pi] * self._num_wps, dtype=np.float64
        )
        self.high = np.array(
            [x_high, y_high, dist_high, np.pi] * self._num_wps, dtype=np.float64
        )

    def step(self, **kwargs):
        sub_goal_manager = self.vehicle.get_attachment(SubGoalAttachment)
        sub_goals = sub_goal_manager.sub_goals[:self.config.observations.sub_goal_num]

        sub_goal_dist_max = self.config.observations.sub_goal_dist_max
        if sub_goal_dist_max is not None:
            ego_pos = self.vehicle.location[:2]
            sub_goals = [s for s in sub_goals if np.linalg.norm(ego_pos - np.array([s.location])) <= sub_goal_dist_max]

        if len(sub_goals) == 0:
            return np.zeros_like(self.low)

        xs = [s.location[0] for s in sub_goals]
        ys = [s.location[1] for s in sub_goals]
        return super().step(xs=xs, ys=ys)

