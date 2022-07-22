import numpy as np

from driver_dojo.core.types import RoadOptionsExtended
import driver_dojo.common.runtime_vars as runtime_vars


class WaypointManager:
    def __init__(self,):
        self._waypoints = None
        self._road_option = None

    def reset(self):
        position = runtime_vars.vehicle.position
        waypoint = runtime_vars.sumo_map.waypoint_on_lane(
            position, runtime_vars.vehicle.laneID
        )

        self._road_option = RoadOptionsExtended.FOLLOW
        self._waypoints = waypoint.next_up_to_distance(
            runtime_vars.config["navigation"]["step_size"],
            runtime_vars.config["navigation"]["max_distance"],
            self._road_option,
        )

    def step(self):
        """Removes navigation that were driven over in the last time step and does a fresh expand of the first
        remaining navigation."""
        if len(self._waypoints) == 0:
            return

        skipped = -1
        for i, waypoint in enumerate(self._waypoints):
            if (
                np.linalg.norm(runtime_vars.vehicle.position - waypoint.position)
                > runtime_vars.config["navigation"]["wp_consume_dist"]
            ):
                break
            else:
                skipped += 1
        last_wp = self._waypoints[-1]
        self._waypoints = self._waypoints[skipped + 1 :]
        if len(self._waypoints) == 0:
            self._waypoints.append(last_wp)

        if len(self._waypoints) > 0:
            # Apply RoadOption and update waypoint sequence
            self._waypoints = self._waypoints[0].next_up_to_distance(
                runtime_vars.config["navigation"]["step_size"],
                runtime_vars.config["navigation"]["max_distance"],
                self.road_option,
            )

    def apply_road_option(self, road_option):
        # Switch lane if command issued, reinitialize waypoint sequence
        if road_option == RoadOptionsExtended.SWITCHLEFT:
            if self._waypoints[0].left is not None:
                self._waypoints = [self._waypoints[0].left]
            self._road_option = RoadOptionsExtended.FOLLOW
        elif road_option == RoadOptionsExtended.SWITCHRIGHT:
            if self._waypoints[0].right is not None:
                self._waypoints = [self._waypoints[0].right]
            self._road_option = RoadOptionsExtended.FOLLOW
        else:
            self._road_option = road_option

        self._waypoints = self._waypoints[0].next_up_to_distance(
            runtime_vars.config["navigation"]["step_size"],
            runtime_vars.config["navigation"]["max_distance"],
            self._road_option,
        )

    @property
    def road_option(self):
        return self._road_option

    @property
    def waypoints(self):
        return self._waypoints
