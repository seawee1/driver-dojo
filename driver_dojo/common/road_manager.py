from __future__ import annotations  # https://stackoverflow.com/questions/33533148/how-do-i-type-hint-a-method-with-the-type-of-the-enclosing-class

from copy import deepcopy
from typing import List, Dict, Optional, Tuple, Any, Union
from dataclasses import dataclass, field
import sumolib
import numpy as np

from driver_dojo.common.waypoint import Waypoint
from driver_dojo.scenarios.basic_scenarios import BasicScenario


class LaneShape:
    def __init__(
            self,
            shape: np.ndarray,
            width: float,
    ):
        self.shape = np.array(shape)

        v = np.array(self.shape[1]) - np.array(self.shape[0])  # First segment direction
        t = np.array([-v[1], v[0]])  # orthogonal direction
        t /= np.linalg.norm(t)  # orthonormal

        offset = width / 2.0
        self.left_border = self.shape + offset * t
        self.right_border = self.shape - offset * t


@dataclass
class RoadShape:
    left_border: np.ndarray
    right_border: np.ndarray

    def __post_init__(self):
        self.polygon = np.array(list(self.left_border) + list(self.right_border[::-1]))


class JunctionNode:
    def __init__(self, sumolib_obj):
        self.sumolib_obj: sumolib.net.node = sumolib_obj
        self.name = sumolib_obj.getID()
        self.type = sumolib_obj.getType()
        self.shape = sumolib_obj.getShape()
        self.area: float = 0.0
        self.route_dist: float = 0.0

        self.incoming: List[RoadNode] = []
        self.outgoing: List[RoadNode] = []
        self.roads: List[RoadNode] = []
        self.lanes: List[LaneNode] = []


class LaneNode:
    def __init__(self, sumolib_obj):
        self.sumolib_obj: sumolib.net.lane = sumolib_obj
        self.name: str = sumolib_obj.getID()
        self.index: int = sumolib_obj.getIndex()
        self.width: float = sumolib_obj.getWidth()
        self.length: float = sumolib_obj.getLength()
        self.shape: LaneShape = LaneShape(
            sumolib_obj.getShape(),
            self.width
        )
        self.road = None
        self.is_part_route: bool = False
        self.left_neigh: Optional[LaneNode] = None
        self.right_neigh: Optional[LaneNode] = None
        self.incoming: List[LaneNode] = []
        self.outgoing: List[LaneNode] = []
        self.function: Optional[str] = None


class RoadNode:
    def __init__(
            self,
            sumolib_obj,
            lanes,
            from_junction,
            to_junction,
    ):
        self.sumolib_obj: sumolib.net.edge = sumolib_obj
        self.name: str = sumolib_obj.getID()
        self.lanes: List[LaneNode] = lanes
        self.from_junction: JunctionNode = from_junction
        self.to_junction: JunctionNode = to_junction
        self.width: float = sum([lane.width for lane in lanes])
        self.length: float = max([lane.length for lane in lanes])
        self.priority: int = sumolib_obj.getPriority()
        self.function: str = sumolib_obj.getFunction()
        self._is_part_route: bool = False

        leftmost_lane = list(filter(lambda l: l.left_neigh is None, lanes))[0]
        rightmost_lane = list(filter(lambda l: l.right_neigh is None, lanes))[0]
        road_shape = RoadShape(
            leftmost_lane.shape.left_border,
            rightmost_lane.shape.right_border,
        )
        self.shape: RoadShape = road_shape

        for lane in self.lanes:  # Link to parent road
            lane.road = self
            lane.function = self.function

        self.junction: Optional[JunctionNode] = None
        self.incoming: List[RoadNode] = []
        self.outgoing: List[RoadNode] = []

    @property
    def is_part_route(self):
        return self._is_part_route

    @is_part_route.setter
    def is_part_route(self, x):
        self._is_part_route = x
        for lane in self.lanes:
            lane.is_part_route = x


class RoadLaneJunctionGraph:
    def __init__(
            self,
            scenario,
    ):
        self.scenario: BasicScenario = scenario
        self.roads: Dict[str, RoadNode] = {}
        self.lanes: Dict[str, LaneNode] = {}
        self.junctions: Dict[str, JunctionNode] = {}
        self.route_partition: RoadLaneJunctionGraph

        # TODO: Internal junction fusing
        for edge in self.scenario.sumo_net.getEdges(withInternal=True):  # Normal edges first
            lanes = []
            lane_index_to_lane = {}
            for lane in edge.getLanes():  # Create initial LaneNode objects
                lane_node = LaneNode(lane)
                self.lanes[lane_node.name] = lane_node
                lanes.append(lane_node)
                lane_index_to_lane[lane.getIndex()] = lane_node

            for lane in lanes:  # Setting left and right neighbors
                if lane.index - 1 in lane_index_to_lane:
                    lane.right_neigh = lane_index_to_lane[lane.index - 1]
                if lane.index + 1 in lane_index_to_lane:
                    lane.left_neigh = lane_index_to_lane[lane.index + 1]

            junctions = []  # Create initial JunctionNode objects connected to current road
            for i, node in enumerate([edge.getFromNode(), edge.getToNode()]):
                name = node.getID()
                if node.getID() not in self.junctions:
                    junction_node = JunctionNode(node)
                    self.junctions[name] = junction_node
                else:
                    junction_node = self.junctions[name]
                junctions.append(junction_node)

            # Create RoadShape for Road
            name = edge.getID()
            road_node = RoadNode(
                edge,
                lanes,
                junctions[0],  # from_node
                junctions[1],  # to_node
            )
            self.roads[name] = road_node

        for junction_id, junction in self.junctions.items():
            for incoming in junction.sumolib_obj.getIncoming():  # Link junction
                junction.incoming.append(self.roads[incoming.getID()])
            for outgoing in junction.sumolib_obj.getOutgoing():
                junction.incoming.append(self.roads[outgoing.getID()])

            conns = junction.sumolib_obj.getConnections()
            for conn in conns:
                from_lane_id = conn.getFromLane().getID()  # Link lanes
                to_lane_id = conn.getToLane().getID()
                via_lane_id = conn.getViaLaneID()

                from_road_id = conn.getFrom().getID()  # Link roads
                to_road_id = conn.getTo().getID()
                if via_lane_id == '':  # Maybe we could skip this, but not sure
                    self.lanes[from_lane_id].outgoing.append(self.lanes[to_lane_id])
                    self.lanes[to_lane_id].incoming.append(self.lanes[from_lane_id])
                    self.roads[from_road_id].outgoing.append(self.roads[to_road_id])
                    self.roads[to_road_id].incoming.append(self.roads[from_road_id])
                else:
                    via_road_id = self.scenario.sumo_net.getLane(conn.getViaLaneID()).getEdge().getID()
                    self.lanes[from_lane_id].outgoing.append(self.lanes[via_lane_id])
                    self.lanes[to_lane_id].incoming.append(self.lanes[via_lane_id])
                    self.lanes[via_lane_id].incoming.append(self.lanes[from_lane_id])
                    self.lanes[via_lane_id].outgoing.append(self.lanes[to_lane_id])
                    self.roads[from_road_id].outgoing.append(self.roads[via_road_id])
                    self.roads[to_road_id].incoming.append(self.roads[via_road_id])
                    self.roads[via_road_id].incoming.append(self.roads[from_road_id])
                    self.roads[via_road_id].outgoing.append(self.roads[to_road_id])

                    junction.roads.append(self.roads[via_road_id])  # Add roads/lanes to junction
                    junction.lanes.append(self.lanes[via_lane_id])
                    self.roads[via_road_id].junction = junction  # Add junction reference

        route_edges = self.scenario.route  # Set is_part_route properties

        for road_id in route_edges:
            self.roads[road_id].is_part_route = True
        internal_part_route = []
        for road_id in route_edges:
            for outgoing in self.roads[road_id].outgoing:
                for outgoing_ in outgoing.outgoing:
                    if outgoing_.is_part_route:
                        outgoing.is_part_route = True
                        break
                    for outgoing__ in outgoing_.outgoing:  # Due to internal junctions
                        if outgoing__.is_part_route:
                            outgoing.is_part_route = True
                            break

        # Remove possible dups in outgoing/incoming
        for comp in [self.roads, self.lanes, self.junctions]:
            for _, c in comp.items():
                c.incoming = list(set(c.incoming))
                c.outgoing = list(set(c.outgoing))

        # Create `route_partition`
        self.route_partition = RoadLaneJunctionGraphPartition(self)


class RoadLaneJunctionGraphPartition:
    def __init__(self, graph):
        self.roads: Dict[str, RoadNode] = {}
        self.lanes: Dict[str, LaneNode] = {}
        self.junctions: Dict[str, JunctionNode] = {}
        for road_id, road in graph.roads.items():
            if road.is_part_route:
                self.roads[road_id] = deepcopy(road)
                for lane in road.lanes:
                    self.lanes[lane.name] = deepcopy(lane)
                if road.junction is not None:
                    self.junctions[road.junction.name] = deepcopy(road.junction)

        # Filter out incoming/outgoing references to components only part of route
        for comp in [self.roads, self.lanes, self.junctions]:
            for _, c in comp.items():
                incoming_or = [x for x in c.incoming if x.is_part_route]
                outgoing_or = [x for x in c.outgoing if x.is_part_route]
                c.incoming = incoming_or
                c.outgoing = outgoing_or

        self.shape: RoadShape

    def get_nearest_lane(self, pos: np.ndarray):
        from shapely.geometry import Point, LineString
        min_d = 13371337  # TODO
        closest_lane = None
        ego_point = Point(pos)
        for lane_id, lane in self.lanes.items():
            d = LineString(lane.shape.shape).distance(ego_point)
            if d < min_d:
                min_d = d
                closest_lane = lane
        return closest_lane






class StreetMap:
    def __init__(self):
        self.scenario = None
        self.graph = None
        self.route_partition = None
        self.lane_graph = None

    def reset(self, scenario: BasicScenario):
        self.scenario = scenario
        self.graph = RoadLaneJunctionGraph(scenario)
        self.route_partition = RoadLaneJunctionGraphPartition(self.graph)

    def waypoint_on_lane(self, location, lane_id):
        lane_node = self.graph.lanes[lane_id]
        lane_position, dist = lane_node.sumolib_obj.getClosestLanePosAndDist(
            location, perpendicular=False
        )
        position = sumolib.geomhelper.positionAtShapeOffset(
            lane_node.sumolib_obj.getShape(), lane_position
        )
        return Waypoint(position, lane_node, lane_position)

# def waypoint_on_lane(self, position, laneID):
    #     lane_node = self.lane_graph[laneID]
    #     lane_position, dist = lane_node.lane.getClosestLanePosAndDist(
    #         position, perpendicular=False
    #     )
    #     position = sumolib.geomhelper.positionAtShapeOffset(
    #         lane_node.lane.getShape(), lane_position
    #     )
    #     return Waypoint(position, lane_node, lane_position)
    #
    # @property
    # def route_edges(self):
    #     return self.scenario.route_edges
    #
    # @property
    # def net(self):
    #     return self.scenario.sumo_net

    # def __init__(self):
    #     self.lane_graph: Dict[str, LaneGraphNode] = dict()
    #     self.nodes: Dict[str, JunctionNode] = dict()
    #
    #     self.scenario = None
    #
    # def reset(self, scenario):
    #     self.scenario = scenario
    #     self.lane_graph = dict()
    #     self.nodes = dict()
    #     self._initialize_lane_graph()
    #     self._initialize_nodes()
    #
    # def _initialize_nodes(self):
    #     nodes = self.scenario.sumo_net.getNodes()
    #     node_ids = [node.getID() for node in nodes]
    #     for i, node_id in enumerate(node_ids):
    #         self.nodes[node_id] = JunctionNode(nodes[i])
    #
    # def _initialize_lane_graph(self):
    #     self.lane_graph = dict()
    #
    #     edges = self.scenario.sumo_net.getEdges(withInternal=True)
    #     edges_by_edgeID = {}
    #     for edge in edges:
    #         lanes = edge.getLanes()
    #         lanes_by_index = {}
    #         for lane in lanes:
    #             lane_index = lane.getIndex()
    #             on_route = True if edge.getID() in self.scenario.route_edges else False
    #             lane_node = LaneGraphNode(lane, on_route)
    #             lanes_by_index[lane_index] = lane_node
    #             self.lane_graph[lane.getID()] = lane_node
    #
    #         # Set left/right attributes
    #         for lane_index, lane_node in lanes_by_index.items():
    #             left_index = lane_index + 1
    #             if left_index in lanes_by_index.keys():
    #                 lanes_by_index[left_index].right = lane_node
    #                 lane_node.left = lanes_by_index[left_index]
    #         edges_by_edgeID[edge.getID()] = lanes_by_index
    #
    #     for edgeID, lanes_by_index in edges_by_edgeID.items():
    #         for lane_index, lane_node in lanes_by_index.items():
    #             # Set outgoing/incoming attributes
    #             outgoing_conns = lane_node.lane.getOutgoing()
    #             for outgoing_conn in outgoing_conns:
    #                 via_laneID = outgoing_conn.getViaLaneID()
    #                 from_laneID = outgoing_conn.getFromLane().getID()
    #                 to_laneID = outgoing_conn.getToLane().getID()
    #
    #                 if via_laneID != '':
    #                     self.lane_graph[from_laneID].outgoing.append(self.lane_graph[via_laneID])
    #                     self.lane_graph[via_laneID].incoming.append(self.lane_graph[from_laneID])
    #                     self.lane_graph[via_laneID].outgoing.append(self.lane_graph[to_laneID])
    #                     self.lane_graph[to_laneID].incoming.append(self.lane_graph[via_laneID])
    #                 else:
    #                     self.lane_graph[from_laneID].outgoing.append(self.lane_graph[to_laneID])
    #                     self.lane_graph[to_laneID].incoming.append(self.lane_graph[from_laneID])
    #
    #                 if outgoing_conn.getFrom().getID() in self.scenario.route_edges and outgoing_conn.getTo().getID() in self.scenario.route_edges:
    #                     self.lane_graph[via_laneID].on_route = True
    #
    #     for k, v in self.lane_graph.items():
    #         self.lane_graph[k].incoming = set(self.lane_graph[k].incoming)
    #         self.lane_graph[k].outgoing = set(self.lane_graph[k].outgoing)
    #
    # def waypoint_on_lane(self, position, laneID):
    #     lane_node = self.lane_graph[laneID]
    #     lane_position, dist = lane_node.lane.getClosestLanePosAndDist(
    #         position, perpendicular=False
    #     )
    #     position = sumolib.geomhelper.positionAtShapeOffset(
    #         lane_node.lane.getShape(), lane_position
    #     )
    #     return Waypoint(position, lane_node, lane_position)
    #
    # @property
    # def route_edges(self):
    #     return self.scenario.route_edges
    #
    # @property
    # def net(self):
    #     return self.scenario.sumo_net
#
#
# class LaneGraphNode:
#     def __init__(self, lane, on_route):
#         self.lane: sumolib.net.lane.Lane = lane
#         self.on_route = on_route
#         self.laneID: str = lane.getID()
#         self.shape = lane.getShape(includeJunctions=False)
#         self.width = lane.getWidth()
#         self.length = lane.getLength()
#         self.is_special = lane.getEdge().isSpecial()
#
#         self.polygon: List[np.ndarray] = []
#         self.left_border: List[np.ndarray] = []
#         self.right_border: List[np.ndarray] = []
#         self._shape_to_polygon()
#
#         # These will be set from the outside
#         self.left: Optional[LaneGraphNode] = None
#         self.right: Optional[LaneGraphNode] = None
#         self.incoming: List[LaneGraphNode] = []
#         self.outgoing: List[LaneGraphNode] = []
#
#     def _shape_to_polygon(self):
#         left = []
#         right = []
#
#         rot = -np.pi / 2.0
#         rotation_mat = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
#         for i in range(len(self.shape)):
#             if i == 0:
#                 p1 = np.array(self.shape[i])
#                 p2 = np.array(self.shape[i + 1])
#                 direction = p2 - p1
#             elif i == len(self.shape) - 1:
#                 p1 = np.array(self.shape[i - 1])
#                 p2 = np.array(self.shape[i])
#                 direction = p2 - p1
#             else:
#                 p1 = np.array(self.shape[i - 1])
#                 p2 = np.array(self.shape[i])
#                 direction1 = p2 - p1
#
#                 p1 = np.array(self.shape[i])
#                 p2 = np.array(self.shape[i + 1])
#                 direction2 = p2 - p1
#                 direction = (direction1 + direction2) / 2.0
#
#             orthonormal_vec = np.dot(rotation_mat, direction) / np.linalg.norm(direction)
#
#             p = np.array(self.shape[i]).tolist()
#             right.append((p + orthonormal_vec * (self.width / 2.0)).tolist())
#             left.append((p - orthonormal_vec * (self.width / 2.0)).tolist())
#
#         self.polygon = left + right[::-1]  # TODO: This doesn't seem to make pyglet.shapes.polygon happy
#         self.left_border = left
#         self.right_border = right
#
#
# class JunctionNodeOld:
#     def __init__(self, sumolib_node):
#         self.node = sumolib_node
#         self.node_id = self.node.getID()
#         self.shape = self.node.getShape()
#
#
# class StreetMap:
#     def __init__(self):
#         self.lane_graph: Dict[str, LaneGraphNode] = dict()
#         self.nodes: Dict[str, JunctionNodeOld] = dict()
#
#         self.scenario = None
#
#     def reset(self, scenario):
#         self.scenario = scenario
#         self.lane_graph = dict()
#         self.nodes = dict()
#         self._initialize_lane_graph()
#         self._initialize_nodes()
#
#     def _initialize_nodes(self):
#         nodes = self.scenario.sumo_net.getNodes()
#         node_ids = [node.getID() for node in nodes]
#         for i, node_id in enumerate(node_ids):
#             self.nodes[node_id] = JunctionNodeOld(nodes[i])
#
#     def _initialize_lane_graph(self):
#         self.lane_graph = dict()
#
#         edges = self.scenario.sumo_net.getEdges(withInternal=True)
#         edges_by_edgeID = {}
#         for edge in edges:
#             lanes = edge.getLanes()
#             lanes_by_index = {}
#             for lane in lanes:
#                 lane_index = lane.getIndex()
#                 on_route = True if edge.getID() in self.scenario.route_edges else False
#                 lane_node = LaneGraphNode(lane, on_route)
#                 lanes_by_index[lane_index] = lane_node
#                 self.lane_graph[lane.getID()] = lane_node
#
#             # Set left/right attributes
#             for lane_index, lane_node in lanes_by_index.items():
#                 left_index = lane_index + 1
#                 if left_index in lanes_by_index.keys():
#                     lanes_by_index[left_index].right = lane_node
#                     lane_node.left = lanes_by_index[left_index]
#             edges_by_edgeID[edge.getID()] = lanes_by_index
#
#         for edgeID, lanes_by_index in edges_by_edgeID.items():
#             for lane_index, lane_node in lanes_by_index.items():
#                 # Set outgoing/incoming attributes
#                 outgoing_conns = lane_node.lane.getOutgoing()
#                 for outgoing_conn in outgoing_conns:
#                     via_laneID = outgoing_conn.getViaLaneID()
#                     from_laneID = outgoing_conn.getFromLane().getID()
#                     to_laneID = outgoing_conn.getToLane().getID()
#
#                     if via_laneID != '':
#                         self.lane_graph[from_laneID].outgoing.append(self.lane_graph[via_laneID])
#                         self.lane_graph[via_laneID].incoming.append(self.lane_graph[from_laneID])
#                         self.lane_graph[via_laneID].outgoing.append(self.lane_graph[to_laneID])
#                         self.lane_graph[to_laneID].incoming.append(self.lane_graph[via_laneID])
#                         if outgoing_conn.getFrom().getID() in self.scenario.route_edges and outgoing_conn.getTo().getID() in self.scenario.route_edges:
#                             self.lane_graph[via_laneID].on_route = True
#                     else:
#                         self.lane_graph[from_laneID].outgoing.append(self.lane_graph[to_laneID])
#                         self.lane_graph[to_laneID].incoming.append(self.lane_graph[from_laneID])
#
#         for k, v in self.lane_graph.items():
#             self.lane_graph[k].incoming = set(self.lane_graph[k].incoming)
#             self.lane_graph[k].outgoing = set(self.lane_graph[k].outgoing)
#
#     def waypoint_on_lane(self, position, laneID):
#         lane_node = self.lane_graph[laneID]
#         lane_position, dist = lane_node.lane.getClosestLanePosAndDist(
#             position, perpendicular=False
#         )
#         position = sumolib.geomhelper.positionAtShapeOffset(
#             lane_node.lane.getShape(), lane_position
#         )
#         return Waypoint(position, lane_node, lane_position)
#
#     @property
#     def route_edges(self):
#         return self.scenario.route_edges
#
#     @property
#     def net(self):
#         return self.scenario.sumo_net
