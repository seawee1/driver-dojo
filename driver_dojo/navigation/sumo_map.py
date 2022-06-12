import sys
import sumolib

from driver_dojo.navigation.waypoint import Waypoint
import driver_dojo.common.state_variables as state_variables


class LaneNode:
    def __init__(self, lane, left, right, incoming, outgoing, on_route, is_goal):
        self.lane = lane
        self.laneID = lane.getID()

        self.left = left
        self.right = right
        self.incoming = incoming
        self.outgoing = outgoing

        if self.left is not None:
            self.left.right = self
        if self.right is not None:
            self.right.left = self
        [node.outgoing.append(self) for node in incoming if self not in node.outgoing]
        [node.incoming.append(self) for node in outgoing if self not in node.incoming]

        self.on_route = on_route
        self.is_goal = is_goal


class SUMOMap:
    def __init__(self):
        self._lane_graph = None

    def init_map(self):
        self.build_lane_graph()

    @property
    def lane_graph(self):
        return self._lane_graph

    def build_lane_graph(self):
        """Builds the lane graph of the ego route.
        For now, we only consider lanes that lie on the ego route and lanes that directly lead off-route (one step).
        """
        self._lane_graph = {}

        # Get first edge of the route
        edge = state_variables.net.getEdge(state_variables.route_edges[0])

        # Get lane types
        lane_types = edge.getType().split("|")
        # Filter lanes to driveable lanes
        # if lane_types[0] != '' and len(lane_types) == len(edge.getLanes()):
        #     # Carla converted data have a type field, e.g. walking|driving|driving to specify, if driveable
        #     lanes = [x for i, x in enumerate(edge.getLanes()) if i > len(lane_types) - 1 or lane_types[i] == 'driving']
        # else:

        # TODO: This way of building the lane graph is an artifact from older times and can probably be implemented much more nicely
        lanes = edge.getLanes()
        lanes_to_sample = lanes
        edges_done = []
        laneIDs_done = []
        while len(lanes_to_sample) != 0:
            connections_to_sample = []
            for lane in lanes_to_sample:
                # Sadly, sumolib.net.lane.getPermissions() is not available and we have to use this to remove disallowed lanes.
                if lane.getWidth() < 2.0:
                    continue

                # Set attributes of navigation
                right_laneID = None
                left_laneID = None
                on_route = True
                edgeID = lane.getEdge().getID()
                laneID = lane.getID()
                if laneID in laneIDs_done:
                    continue
                is_goal = edgeID == state_variables.route_edges[-1]
                for lane2 in lanes_to_sample:
                    if lane2.getIndex() == lane.getIndex() - 1:
                        right_laneID = lane2.getID()
                    if lane2.getIndex() == lane.getIndex() + 1:
                        left_laneID = lane2.getID()

                # Get incoming/outgoing connections
                outgoing_laneIDs = [x.getViaLaneID() for x in lane.getOutgoing()]
                incoming_laneIDs = [x.getID() for x in lane.getIncoming(True)]

                if not is_goal:
                    # Append all outgoing connections
                    for conn in lane.getOutgoing():
                        connections_to_sample.append((conn, False))

                # Flag edge as already sampled in order to avoid circular execution
                edge_id = lane.getEdge().getID()
                if edge_id not in edges_done:
                    edges_done.append(edge_id)

                left = (
                    self._lane_graph[left_laneID]
                    if left_laneID in self._lane_graph
                    else None
                )
                right = (
                    self._lane_graph[right_laneID]
                    if right_laneID in self._lane_graph
                    else None
                )
                incoming = [
                    self._lane_graph[laneID]
                    for laneID in incoming_laneIDs
                    if laneID in self._lane_graph
                ]
                outgoing = [
                    self._lane_graph[laneID]
                    for laneID in outgoing_laneIDs
                    if laneID in self._lane_graph
                ]
                self._lane_graph[laneID] = LaneNode(
                    lane, left, right, incoming, outgoing, on_route, is_goal
                )
                laneIDs_done.append(laneID)

            # lanes_to_sample have all been sampled, so empty the list
            lanes_to_sample = []

            # Continue with connections
            for conn, is_internal in connections_to_sample:
                # Get the lane object of this connection
                lane = state_variables.net.getLane(conn.getViaLaneID())

                # Check if connected lane is on route
                on_route = False
                to_edgeID = conn.getToLane().getEdge().getID()
                if (
                    to_edgeID in state_variables.route_edges
                    and to_edgeID not in edges_done
                ):
                    on_route = True

                # Check other variables
                right_laneID = None
                left_laneID = None
                laneID = lane.getID()
                if laneID in laneIDs_done:
                    continue
                is_goal = False

                # It is tricky to identify changeable lanes for junction lanes.
                # We do this be comparing from- and to-lane indices.
                for conn2, is_internal2 in connections_to_sample:
                    if is_internal != is_internal2:
                        continue

                    if (
                        not is_internal
                        and conn2.getTo().getID() == conn.getTo().getID()
                    ):
                        conn2_fromIndex = conn2.getFromLane().getIndex()
                        conn_fromIndex = conn.getFromLane().getIndex()
                        if conn_fromIndex == conn2_fromIndex - 1:
                            left_laneID = conn2.getViaLaneID()
                        if conn_fromIndex == conn2_fromIndex + 1:
                            right_laneID = conn2.getViaLaneID()

                # Append to lanes_to_sample list
                if (
                    to_edgeID in state_variables.route_edges
                    and to_edgeID not in edges_done
                ):
                    lanes_to_sample.append(conn.getToLane())
                    # Also append lanes not directly connected (long north-south lane fix)
                    lanes2 = conn.getToLane().getEdge().getLanes()
                    lane_types = conn.getToLane().getEdge().getType().split("|")
                    if not len(lane_types) == 1 and lane_types[0] == "":
                        lanes2 = [
                            x
                            for i, x in enumerate(lanes2)
                            if lane_types[i] == "driving"
                        ]
                    for lane2 in lanes2:
                        if lane2 not in lanes_to_sample:
                            lanes_to_sample.append(lane2)

                # Internal junctions are weird
                internal_junction = (
                    state_variables.net.getLane(laneID).getOutgoing()[0].getViaLaneID()
                    != ""
                )
                if internal_junction:
                    outgoing_laneIDs = [
                        state_variables.net.getLane(laneID)
                        .getOutgoing()[0]
                        .getViaLaneID()
                    ]
                    connections_to_sample.append(
                        (state_variables.net.getLane(laneID).getOutgoing()[0], True)
                    )
                else:
                    outgoing_laneIDs = [conn.getToLane().getID()]
                incoming_laneIDs = [x.getID() for x in lane.getIncoming()]

                left = (
                    self._lane_graph[left_laneID]
                    if left_laneID in self._lane_graph
                    else None
                )
                right = (
                    self._lane_graph[right_laneID]
                    if right_laneID in self._lane_graph
                    else None
                )
                incoming = [
                    self._lane_graph[laneID]
                    for laneID in incoming_laneIDs
                    if laneID in self._lane_graph
                ]
                outgoing = [
                    self._lane_graph[laneID]
                    for laneID in outgoing_laneIDs
                    if laneID in self._lane_graph
                ]
                self._lane_graph[laneID] = LaneNode(
                    lane, left, right, incoming, outgoing, on_route, is_goal
                )
                laneIDs_done.append(laneID)
        self._lane_graph = self._lane_graph

    def waypoint_on_lane(self, position, laneID):
        if laneID not in self.lane_graph.keys():
            print("Stop!!!!")
            # try:
            #     self.build_lane_graph()
            #     print(self.lane_graph.keys())
            #     lane_node = self.lane_graph[laneID]
            # except Exception as ex:
            #     print("Oh oh! This shouldn't happen...")
            #     exit()

        lane_node = self.lane_graph[laneID]
        lane_position, dist = lane_node.lane.getClosestLanePosAndDist(
            position, perpendicular=False
        )
        position = sumolib.geomhelper.positionAtShapeOffset(
            lane_node.lane.getShape(), lane_position
        )
        return Waypoint(position, lane_node, lane_position)

    @property
    def route_edges(self):
        return state_variables.route_edges

    @property
    def net(self):
        return state_variables.net


def parse_route(route_file, route_id, standalone=True):
    """Extracts a list of edgeIDs from a route file for a specific route.
    standalone = True will also parse routes which do not have attached vehicle.
    """
    known_ids = set()

    def unique_id(cand, index=0):
        cand2 = cand
        if index > 0:
            cand2 = "%s#%s" % (cand, index)
        if cand2 in known_ids:
            return unique_id(cand, index + 1)
        else:
            known_ids.add(cand2)
            return cand2

    def filterEdges(edges, keep):
        if keep is None:
            return edges
        else:
            return [e for e in edges if e in keep]

    keep = None
    if standalone:
        for route in sumolib.output.parse(route_file, "route"):
            if route.id == route_id:
                return filterEdges(route.edges.split(), keep)


def find_lanes_through_edges(net, edge_ids, start_lane_id):
    """Finds a random sequence of lanes that lead through a list of edges."""
    edges = []
    for e in edge_ids:
        if net.hasEdge(e):
            edges.append(net.getEdge(e))
        else:
            sys.stderr.write("Warning: unknown edge '%s'\n" % e)

    lane_ids = [start_lane_id]
    for edge_id in edge_ids[1:]:
        lane = net.getLane(lane_ids[-1])
        cons = lane.getOutgoing()

        for con in cons:
            if con.getToLane().getEdge().getID() == edge_id:
                lane_ids.append(con.getViaLaneID())
                lane_ids.append(con.getToLane().getID())
    return lane_ids
