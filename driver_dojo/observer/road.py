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
                #-np.pi
                0,
            ] * 9,
            dtype=np.float64,
        )
        self.high = np.array(
            [
                np.inf,
                np.inf,
                #np.pi
                1,
            ] * 9,
            dtype=np.float64,
        )

    def step(self):
        from driver_dojo.common import utils
        veh_state = self.vehicle.state
        def nearest_point_on_lane(ego_point, lane_node):
            return shapely.ops.nearest_points(ego_point, shapely.geometry.LineString(lane_node.shape.shape))[1]

        ego_point = utils.transform_to_sumo(veh_state.location[0], veh_state.location[1], veh_state.rotation[1], veh_state.length/2.0+0.5)
        ego_point = shapely.geometry.Point(ego_point)
        #ego_point = shapely.geometry.Point(veh_state.location[:2])
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
        nextnext_lane = None
        nextnext_lane_left = None
        nextnext_lane_right = None
        if next_lane is not None and len(next_lane.outgoing) > 0:
            nextnext_lane = next_lane.outgoing[0]
            nextnext_lane_left = nextnext_lane.left_neigh
            nextnext_lane_right = nextnext_lane.right_neigh


        obs = np.zeros_like(self.low)
        none_mask = []
        for i, lane_node in enumerate([
            cur_lane_left, cur_lane, cur_lane_right,
            next_lane_left, next_lane, next_lane_right,
            nextnext_lane_left, nextnext_lane, nextnext_lane_right,
        ]):
            if lane_node is None:
                none_mask.append(i)
                continue
            p = nearest_point_on_lane(ego_point, lane_node)

            if self.config.observations.relative_to_ego:
                theta = utils.wrap_to_pi(-self.vehicle.rotation[1]) + np.radians(90)
                rot = np.array(
                    [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
                )
                xy = np.array([p.x - veh_state.location[0], p.y - veh_state.location[1]])
                xy = np.dot(rot, xy.T).T
                obs[i*3] = xy[0]
                obs[i*3 + 1] = xy[1]
            else:
                theta = self.vehicle.rotation
                obs[i*3] = p.x
                obs[i*3 + 1] = p.y

            obs[i*3 + 2] = 0 if len(lane_node.outgoing) == 0 else 1
            # angle = utils.wrap_to_pi(theta)
            # angle_wp = utils.wrap_to_pi(np.arctan2(obs[i*3 + 1], obs[i*3]))  # Match angles
            # match_angle = utils.wrap_to_pi(angle_wp - angle)
            # obs[i * 3 + 2] = match_angle
            #print(obs[i*2], obs[i*2+1], match_angle)

        obs = self._normalize_obs(obs)

        for i in none_mask:
            obs[i*2] = 0.0
            obs[i*2 + 1] = 0.0

        return obs

    def explain(self):
        return
