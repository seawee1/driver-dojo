import numpy as np
from PathPlanning.CubicSpline import cubic_spline_planner

from driver_dojo.core.types import RoadOptionsExtended


class WaypointAttachment:
    def __init__(self, config, vehicle, street_map):
        self.config = config
        self.vehicle = vehicle
        self.street_map = street_map
        self._wps = None
        self._cur_road_option = None
        self._nav_config = self.config.navigation
        self._ref_wp = None

    def reset(self):
        self._cur_road_option = RoadOptionsExtended.FOLLOW

        location = self.vehicle.state.location  # First reference waypoints is ego's location
        lane_id = self.vehicle.state.lane_id
        ego_wp = self.street_map.waypoint_on_lane(
            location, lane_id
        )
        self._ref_wp = ego_wp
        self._wps = self._sample_waypoints()

    def step(self):
        self._wps = self._sample_waypoints()

    def apply_road_option(self, road_option):
        # Switch lane if command issued, reinitialize waypoint sequence
        if road_option == RoadOptionsExtended.SWITCHLEFT:
            if self._wps[0].left is not None:
                self._ref_wp = self._wps[0].left
            self._cur_road_option = RoadOptionsExtended.FOLLOW
        elif road_option == RoadOptionsExtended.SWITCHRIGHT:
            if self._wps[0].right is not None:
                self._ref_wp = self._wps[0].right
            self._cur_road_option = RoadOptionsExtended.FOLLOW
        else:
            # TODO: inside Waypoint.next_up_to_distance() we currently pick left/right waypoint at random
            self._cur_road_option = road_option

        self._sample_waypoints()

    def _sample_waypoints(self):
        veh_state = self.vehicle.state  # Project location to front of car
        x, y = veh_state.location[:2]
        rot = veh_state.rotation[1]
        forw = veh_state.length / 2.0
        x = x + np.cos(rot) * forw
        y = y + np.sin(rot) * forw
        veh_location = np.array([x, y])

        sampling_wp = self.street_map.waypoint_on_lane(veh_location, self._ref_wp.lane_node.laneID)  # Get the nearest location to the car on the assigned lane
        wps = sampling_wp.next_up_to_distance(  # Sample some waypoints
            self._nav_config.step_size,
            self._nav_config.initial_dist + self._nav_config.num_waypoints * self._nav_config.step_size + 10.0,
            self._cur_road_option,
        )

        wps = [  # Subsample based on NavigationConfig
            wp for wp in wps
            if np.linalg.norm(veh_location - wp.location) > self._nav_config.initial_dist
               and np.linalg.norm(veh_state.location[:2] - wp.location) > self._nav_config.initial_dist  # Prevents a bug for vehicles that are longer than 2*initial_dist
        ]
        wps = wps[:self._nav_config.num_waypoints]
        self._ref_wp = wps[0]

        return wps

    @property
    def cur_road_option(self):
        return self._cur_road_option

    @property
    def waypoints(self):
        return self._wps

    def cubic_spline_course(self, position, res=0.1):
        xs = [position[0]] + [wp.location[0] for wp in self.waypoints]
        ys = [position[1]] + [wp.location[1] for wp in self.waypoints]

        xs, ys, yaw, k, s = cubic_spline_planner.calc_spline_course(xs, ys, ds=res)
        return xs, ys, yaw, k, s


