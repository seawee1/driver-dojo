import numpy as np
import sumolib

from driver_dojo.common.waypoint import SimpleWaypoint


class SubGoalAttachment:
    def __init__(self, config, vehicle, street_map):
        self.config = config
        self.vehicle = vehicle
        self.street_map = street_map
        self._nav_config = self.config.navigation
        self._sub_goals = None

    def reset(self):
        self._sub_goals = []

        for i, edgeID in enumerate(self.street_map.scenario.route):
            edge = self.street_map.scenario.sumo_net.getEdge(edgeID)
            start_pos = sumolib.geomhelper.positionAtShapeOffset(edge.getShape(), 0.0)
            end_pos = sumolib.geomhelper.positionAtShapeOffset(
                edge.getShape(), edge.getLength()
            )

            if i > 0 and not self.config.simulation.subgoals_only_after:
                self._sub_goals.append(SimpleWaypoint(np.array(start_pos)))
            self._sub_goals.append(SimpleWaypoint(np.array(end_pos)))

        if self._nav_config.only_end_goal:
            self._sub_goals = self._sub_goals[-1]

    def step(self):
        """Removes navigation that were driven over in the last time step and does a fresh expand of the first
        remaining navigation."""
        veh_state = self.vehicle.state

        if len(self._sub_goals) == 0:
            return

        remove_idx = -1
        for i, wp in enumerate(self._sub_goals):
            if (
                np.linalg.norm(veh_state.location[:2] - wp.location) > self._nav_config.sub_goal_consume_dist
            ):
                break
            else:
                remove_idx += 1

        self._sub_goals = self._sub_goals[remove_idx+1:]

    @property
    def sub_goals(self):
        return self._sub_goals
