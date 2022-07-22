import sys
import sumolib

from driver_dojo.navigation.waypoint import Waypoint
import driver_dojo.common.runtime_vars as runtime_vars


class LaneNode:
    def __init__(self, lane):
        self.lane = lane
        self.laneID = lane.getID()

        self.left = None
        self.right = None
        self.incoming = []
        self.outgoing = []


class SUMOMap:
    def __init__(self):
        self.lane_graph = None

    def init_map(self):
        self.build_lane_graph()

    def build_lane_graph(self):
        self.lane_graph = {}

        edges = runtime_vars.net.getEdges(withInternal=True)
        edges_by_edgeID = {}
        for edge in edges:
            lanes = edge.getLanes()
            lanes_by_index = {}
            for lane in lanes:
                lane_index = lane.getIndex()
                lane_node = LaneNode(lane)
                lanes_by_index[lane_index] = lane_node
                self.lane_graph[lane.getID()] = lane_node

            # Set left/right attributes
            for lane_index, lane_node in lanes_by_index.items():
                left_index = lane_index + 1
                if left_index in lanes_by_index.keys():
                    lanes_by_index[left_index].right = lane_node
                    lane_node.left = lanes_by_index[left_index]
            edges_by_edgeID[edge.getID()] = lanes_by_index

        for edgeID, lanes_by_index in edges_by_edgeID.items():
            for lane_index, lane_node in lanes_by_index.items():
                # Set outgoing/incoming attributes
                outgoing_conns = lane_node.lane.getOutgoing()
                for outgoing_conn in outgoing_conns:
                    via_laneID = outgoing_conn.getViaLaneID()
                    from_laneID = outgoing_conn.getFromLane().getID()
                    to_laneID = outgoing_conn.getToLane().getID()

                    if via_laneID != '':
                        self.lane_graph[from_laneID].outgoing.append(self.lane_graph[via_laneID])
                        self.lane_graph[via_laneID].incoming.append(self.lane_graph[from_laneID])
                        self.lane_graph[via_laneID].outgoing.append(self.lane_graph[to_laneID])
                        self.lane_graph[to_laneID].incoming.append(self.lane_graph[via_laneID])
                    else:
                        self.lane_graph[from_laneID].outgoing.append(self.lane_graph[to_laneID])
                        self.lane_graph[to_laneID].incoming.append(self.lane_graph[from_laneID])

        for k, v in self.lane_graph.items():
            self.lane_graph[k] = set(v)

    def waypoint_on_lane(self, position, laneID):
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
        return runtime_vars.route_edges

    @property
    def net(self):
        return runtime_vars.net


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
