import numpy as np
import sumolib

from driver_dojo.core.types import RoadOptionsExtended
import driver_dojo.common.runtime_vars as runtime_vars


class SimpleWaypoint:
    def __init__(self, position):
        self.position = position


class Waypoint:
    def __init__(self, position, lane_node, lane_position):
        self.position = (
            np.array(position) if not isinstance(position, np.ndarray) else position
        )
        self.lane_node = lane_node
        self.lane_position = lane_position

    def next(self, step_size):
        """Give the next waypoint from the current waypoint.
        Returns a list, with one waypoint for every possible next waypoint possible (i.e. when a road splits)
        """
        waypoint_next = []

        x, y = sumolib.geomhelper.positionAtShapeOffset(
            self.lane_node.lane.getShape(), self.lane_position + step_size
        )
        position = np.array([x, y])

        # The next waypoint is on the current lane
        if np.linalg.norm(position - self.position) >= step_size - 0.1:
            waypoint = Waypoint(
                position, self.lane_node, self.lane_position + step_size
            )
            waypoint_next.append(waypoint)
            return waypoint_next

        # The next waypoint is on one of the next lanes
        for to_lane_node in self.lane_node.outgoing:
            to_lane = to_lane_node.lane

            offset_step = 0.1
            lane_offset = offset_step
            position_old = np.array([0.0, 0.0])
            while True:
                x, y = sumolib.geomhelper.positionAtShapeOffset(
                    to_lane.getShape(), lane_offset
                )
                position = np.array([x, y])
                if np.all(position == position_old):
                    #print('Warning: Skipped lane during waypoint calculation, which is ill-defined! '
                    #      'Please decrease step_size!')
                    break

                if np.linalg.norm(position - self.position) >= step_size:
                    break

                position_old = position
                lane_offset += offset_step

            # Create the waypoint
            waypoint = Waypoint(position, to_lane_node, lane_offset)
            waypoint_next.append(waypoint)

        return waypoint_next

    def next_up_to_distance(self, step_size, max_distance, road_option):
        if (
            road_option == RoadOptionsExtended.SWITCHRIGHT
            or road_option == RoadOptionsExtended.SWITCHLEFT
        ):
            road_option = RoadOptionsExtended.FOLLOW

        waypoints = [self]

        branched = False
        distance = 0.0
        while distance < max_distance:
            waypoint_next = waypoints[-1].next(step_size)

            # We reached the goal or a dead end
            if len(waypoint_next) == 0:
                break
            # There is only one next waypoint to follow
            elif len(waypoint_next) == 1:
                waypoints.append(waypoint_next[0])
                distance += step_size
            else:
                # We have to make a decision here. If we already branched or just want to follow the route,
                # take the branch that makes us follow the planned route.
                if branched or road_option == RoadOptionsExtended.FOLLOW:
                    for waypoint in waypoint_next:
                        if waypoint.on_route:
                            waypoints.append(waypoint)
                            branched = True
                            break
                # Else, we take a branch that takes us off-route.
                elif (
                    road_option == RoadOptionsExtended.LEFT
                    or road_option == RoadOptionsExtended.RIGHT
                ):
                    # TODO: Left and right RoadOption. Right now we randomly select either of those navigation.
                    for waypoint in waypoint_next:
                        if not waypoint.on_route:
                            waypoints.append(waypoint)
                            branched = True
                            break
                distance += step_size

        return waypoints

    def _nearest_waypoint_on_lane(self, lane_node):
        if lane_node is None:
            return None

        lane_position, _ = lane_node.lane.getClosestLanePosAndDist(
            self.position, perpendicular=False
        )
        position = sumolib.geomhelper.positionAtShapeOffset(
            lane_node.lane.getShape(), lane_position
        )
        waypoint = Waypoint(position, lane_node, lane_position)
        return waypoint

    @property
    def on_route(self):
        edgeID = self.lane_node.lane.getEdge().getID()
        if edgeID in runtime_vars.route_edges:
            return True
        else:
            # We are on an internal lane
            # TODO: Might cause problems if we have two internal lanes in succession
            for outgoing in self.lane_node.outgoing:
                outgoing_edgeID = outgoing.lane.getEdge().getID()
                if outgoing in runtime_vars.route_edges:
                    return True
        return False

    @property
    def on_goal(self):
        edgeID = self.lane_node.lane.getEdge().getID()
        return edgeID == runtime_vars.route_edges[-1]

    @property
    def left(self):
        """Returns an equivalent waypoint on the left lane."""
        return self._nearest_waypoint_on_lane(self.lane_node.left)

    @property
    def right(self):
        """ Returns an equivalent waypoint on the right lane."""
        return self._nearest_waypoint_on_lane(self.lane_node.right)

    @property
    def road_options(self):
        road_options = []

        # Check if we can switch lanes
        if self.left is not None:
            road_options.append(RoadOptionsExtended.SWITCHLEFT)
        if self.right is not None:
            road_options.append(RoadOptionsExtended.SWITCHRIGHT)

        step_size = 1.0
        waypoint_next = self.next(step_size)
        cur_dist = step_size

        if len(waypoint_next) == 0:
            return road_options

        road_options.append(RoadOptionsExtended.FOLLOW)
        while True:
            if len(waypoint_next) == 0:
                break
            elif len(waypoint_next) == 1:
                waypoint_next = waypoint_next[0].next(step_size)
                cur_dist += step_size
                if cur_dist > 50.0:
                    break
            else:
                # TODO: Correctly handle left and right road options
                road_options.append(RoadOptionsExtended.LEFT)
                road_options.append(RoadOptionsExtended.RIGHT)
                break

        return road_options
