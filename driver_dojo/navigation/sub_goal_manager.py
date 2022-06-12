import numpy as np
import sumolib

from driver_dojo.navigation.waypoint import SimpleWaypoint
import driver_dojo.common.state_variables as state_variables


class SubGoalManager:
    def __init__(self):
        self.navigation_config = state_variables.config["navigation"]
        self._sub_goals = None

    def reset(self):
        sub_goals = []
        for i, edgeID in enumerate(state_variables.sumo_map.route_edges):
            edge = state_variables.sumo_map.net.getEdge(edgeID)
            start_pos = sumolib.geomhelper.positionAtShapeOffset(edge.getShape(), 0.0)
            end_pos = sumolib.geomhelper.positionAtShapeOffset(
                edge.getShape(), edge.getLength()
            )

            if i > 0 and not state_variables.config.simulation.subgoals_only_after:
                sub_goals.append(SimpleWaypoint(np.array(start_pos)))
            sub_goals.append(SimpleWaypoint(np.array(end_pos)))

        if self.navigation_config["only_end_goal"]:
            sub_goals = [sub_goals[-1]]

        self._sub_goals = sub_goals

    def step(self):
        """Removes navigation that were driven over in the last time step and does a fresh expand of the first
        remaining navigation."""
        if len(self._sub_goals) == 0:
            return

        skipped = -1
        for i, waypoint in enumerate(self._sub_goals):
            if (
                np.linalg.norm(state_variables.vehicle.position - waypoint.position)
                > self.navigation_config["sub_goal_consume_dist"]
            ):
                break
            else:
                skipped += 1

        self._sub_goals = self._sub_goals[skipped + 1 :]

    @property
    def sub_goals(self):
        return self._sub_goals
