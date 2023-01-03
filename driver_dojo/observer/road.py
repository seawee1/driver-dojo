# Nearest lane center dist

# Alignment with road (angle)
# Road shape derivative
import numpy as np
import shapely

from driver_dojo.observer import BaseObserver

class RoadObserver(BaseObserver):
    def __init__(self, config, vehicle, traffic_manager, street_map):
        super().__init__(config, vehicle, traffic_manager)
        self._street_map = street_map

        self.low = np.array(
            [
                -np.inf,
                -np.inf,
            ] * 6,
            dtype=np.float64,
        )
        self.high = np.array(
            [
                np.inf,
                np.inf
            ] * 6,
            dtype=np.float64,
        )

    def step(self):
        veh_state = self.vehicle.state
        def nearest_point_on_lane(ego_point, lane_node):
            return shapely.ops.nearest_points(ego_point, shapely.geometry.LineString(lane_node.shape.shape))[1]

        ego_point = shapely.geometry.Point(veh_state.location[:2])
        partition_graph = self._street_map.route_partition

        cur_lane = partition_graph.get_nearest_lane(ego_point)
        cur_lane_left = cur_lane.left_neigh
        cur_lane_right = cur_lane.right_neigh
        assert len(cur_lane.outgoing) <= 1  # TODO: Hope this doesn't happen
        next_lane = None
        next_lane_left = None
        next_lane_right = None
        if len(cur_lane.outgoing) > 0:
            next_lane = cur_lane.outgoing[0]
            next_lane_left = next_lane.left_neigh
            next_lane_right = next_lane.right_neigh

        obs = np.zeros_like(self.low)
        none_mask = []
        for i, lane_node in enumerate([cur_lane_left, cur_lane, cur_lane_right, next_lane_left, next_lane, next_lane_right]):
            if lane_node is None:
                none_mask.append(i)
                continue
            p = nearest_point_on_lane(ego_point, lane_node)
            obs[i*2] = p.x if not self.config.observations.relative_to_ego else p.x - ego_point.x
            obs[i*2 + 1] = p.y if not self.config.observations.relative_to_ego else p.y - ego_point.y

        obs = self._normalize_obs(obs)

        for i in none_mask:
            obs[i*2] = 0.0
            obs[i*2 + 1] = 0.0

        return obs

    def explain(self):
        return
