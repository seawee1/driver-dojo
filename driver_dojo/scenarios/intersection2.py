import subprocess
from typing import List, Dict
import numpy as np
from pyclothoids import Clothoid

from driver_dojo.scenarios.basic_scenarios import BasicScenario
from driver_dojo.scenarios.plainxml import Edge, Node, Connection, write_plain_xml
from driver_dojo.core.config import Config
import sumolib


class IntersectionScenario(BasicScenario):
    def __init__(
            self,
            base_path,
            scenario_config,
            sim_config,
            net_seed,
            traffic_seed,
            task_seed,
    ):

        super().__init__(
            base_path,
            scenario_config,
            sim_config,
            net_seed,
            traffic_seed,
            task_seed,
            generate=False
        )
        assert self._task is not None  # We need this for Intersection

        self._crossing_style = self.task_rng.choice(scenario_config.kwargs['crossing_style']).lower()
        assert self._crossing_style in ['major', 'minor', 'rbl']
        self._crossing_style_to_conn_state = {  # Defines the SUMO connection state (conn.getState()) that should be part of the ego route
            'major': 'M',
            'minor': 'm',
            'rbl': None,
        }

        self._node_name: str = 'main'  # TODO: make this more generic
        self._shape_step: float = 1.0
        self._nodes: Dict[str, Node] = {}
        self._edges: Dict[str, Edge] = {}
        self._route_candidates = []

        self._generate_init()

    @property
    def _junction_connections(self):
        conns = []
        for conn in self.sumo_net.getNode(self._node_name).getConnections():  # Get all junction connections
            if conn.getFrom().getFunction() != 'internal' and conn.getTo().getFunction() != 'internal':
                conns.append(conn)
        return conns

    @property
    def ego_init_edges(self):
        return [x for x, y in self._route_candidates]
        # ego_init_edges = []
        # required_conn_state = self._crossing_style_to_conn_state[self._crossing_style]
        # for conn in self._junction_connections:  # Get all junction connections
        #     if required_conn_state is None or conn.getState() == required_conn_state:
        #         ego_init_edges.append(conn.getFrom().getID())
        # assert len(ego_init_edges) > 0, f'{self.net_seed}, {self.traffic_seed}'
        # return ego_init_edges

    def ego_routing_strategy(self, from_edge: str) -> List[str]:
        to_edges = []
        for x, y in self._route_candidates:
            if x == from_edge:
                to_edges.append(y)
        self._scenario_config.ego_to_edges = to_edges
        return super().ego_routing_strategy(from_edge)
        # ego_target_edges = []
        # required_conn_state = self._crossing_style_to_conn_state[self._crossing_style]
        # for conn in self._junction_connections:  # Get all junction connections
        #     if conn.getFrom().getID() == from_edge and (conn.getState() == required_conn_state or required_conn_state is None):
        #         ego_target_edges.append(conn.getTo().getID())
        #
        # to_edge = self.traffic_rng.choice(ego_target_edges)
        # optimal_routes = self.sumo_net.getOptimalPath(self.sumo_net.getEdge(from_edge), self.sumo_net.getEdge(to_edge))
        # assert optimal_routes[0] is not None  # Otherwise, this route is impossible with the underlying map
        # route_edge_ids = [x.getID() for x in optimal_routes[0]]
        # return route_edge_ids

    def generate_map(self, params, base_path):
        n_roads, n_lanes_in, n_lanes_out, road_sep, road_angles, t_inner, t_outer, airline_dist = params#, priorities = params

        node_type = 'right_before_left' if self._crossing_style == 'rbl' else 'priority'
        node_pos = [0.0, 0.0]
        main_node = Node(self._node_name, 0.0, 0.0, 0.0, type=node_type, rightOfWay='default')
        self._nodes[self._node_name] = main_node

        for i in range(n_roads):
            x, y = node_pos[0] + np.cos(road_angles[i]) * airline_dist[i], node_pos[1] + np.sin(road_angles[i]) * airline_dist[i]
            self._nodes[str(i)] = Node(str(i), x, y, 0.0)

        for i in range(n_roads):
            node = self._nodes[str(i)]
            cloth = Clothoid.G1Hermite(main_node.x, main_node.y, t_inner[i], node.x, node.y, t_outer[i])
            num_shape_samples = int(cloth.length // self._shape_step)

            if n_lanes_in[i] > 0:
                shape = cloth.Reverse().SampleXY(num_shape_samples)
                self._edges[f'{i}_in'] = Edge(f'{i}_in', str(i), self._node_name, numLanes=n_lanes_in[i], shape=shape)#, priority=priorities[i])

            if n_lanes_out[i] > 0:
                if road_sep[i] > 0.0:
                    orth_angle = road_angles[i] - np.pi / 2
                    x_off, y_off = np.cos(orth_angle) * road_sep[i], np.sin(orth_angle) * road_sep[i]
                    cloth = cloth.Translate(x_off, y_off)
                shape = cloth.SampleXY(num_shape_samples)
                self._edges[f'{i}_out'] = Edge(f'{i}_out', self._node_name, str(i), numLanes=n_lanes_out[i], shape=shape)#, priority=priorities[i])

        nodes_xml_path, edges_xml_path, conns_xml_path = write_plain_xml(base_path, self._nodes.values(), self._edges.values(), conns=None)
        p = subprocess.Popen(
            self._netconvert_cmd(nodes_xml_path, edges_xml_path, base_path + '.net.xml')#, stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )
        p.wait()

    def task_specifics(self):
        from_edges, to_edges = self._task_from_to_edges(self._map_params)  # Sample possible from/to edges, based on task
        if len(from_edges) == 0:
            return False

        self._route_candidates = self._filter_route_candidates(self.sumo_net, from_edges, to_edges)
        if len(self._route_candidates) == 0:
            return False
        return True

    def _filter_route_candidates(self, sumo_net, from_edges, to_edges):
        required_conn_state = self._crossing_style_to_conn_state[self._crossing_style]
        route_candidates = []
        for conn in self._junction_connections:
            if required_conn_state is None or conn.getState() == required_conn_state:
                conn_from = int(conn.getFrom().getID().split('_')[0])
                conn_to = int(conn.getTo().getID().split('_')[0])
                real = False
                for from_edge, to_edge in zip(from_edges, to_edges):
                    if conn_from == from_edge and conn_to == to_edge:
                        real = True
                        break
                if real:
                    route_candidates.append((conn.getFrom().getID(), conn.getTo().getID()))
        return route_candidates

    def _netconvert_cmd(self, nodes_xml_path, edges_xml_path, net_path):
        return [
            'netconvert',
            '-n',
            nodes_xml_path,
            '-e',
            edges_xml_path,
            '--junctions.corner-detail',
            str(20),
            '--junctions.internal-link-detail',
            str(50),
            '--no-turnarounds',
            'true',
            '--output-file',
            net_path,
            '--check-lane-foes.all',
            'true'
        ]

    def sample_map_parameters(self, rng: np.random.Generator):
        def indices_with_lanes(n_lanes_lst):
            return [i for i in range(len(n_lanes_lst)) if n_lanes_lst[i] > 0]

        num_lanes_range = [0, 1, 2]
        num_roads_cand = [3, 4]
        n_roads = rng.choice(num_roads_cand)
        n_lanes_in = rng.choice(num_lanes_range, size=n_roads, p=[0.1, 0.45, 0.45])
        n_lanes_out = rng.choice(num_lanes_range, size=n_roads, p=[0.1, 0.45, 0.45])

        for i, (n_in, n_out) in enumerate(zip(n_lanes_in, n_lanes_out)):  # Repair the case where we have a road with neither inward nor outward lanes
            if n_in == 0 and n_out == 0:
                rng.choice([n_lanes_in, n_lanes_out])[i] = rng.choice(num_lanes_range[1:])  # Set either inward or outward lane number to something > 0

        for lst in [n_lanes_in, n_lanes_out]: # Repair the case where less than two roads lead into/out of the junction
            indices = indices_with_lanes(lst)
            for _ in range(max(0, 2 - len(indices))):
                lst[rng.choice(indices)] = rng.choice(num_lanes_range[1:])

        road_sep = [rng.uniform(0.2, 2.0) if rng.uniform(0.0, 1.0) >= 0.5 else 0.0 for i in range(n_roads)]  # Randomized separation between opposing road directions
        angle_var = 0.8 * (365.0 / n_roads / 2)
        road_angles = [
            np.radians(i * 365.0 / n_roads + rng.integers(-angle_var, angle_var)) for i in range(n_roads)
        ]
        t_inner = [rng.uniform(road_angle - np.pi / 16, road_angle + np.pi / 16) for road_angle in road_angles]
        t_outer = [rng.uniform(road_angle - np.pi / 4, road_angle + np.pi / 4) for road_angle in road_angles]
        airline_dist = [rng.integers(50.0, 100.0) for i in range(n_roads)]
        return n_roads, n_lanes_in, n_lanes_out, road_sep, road_angles, t_inner, t_outer, airline_dist

    def _task_from_to_edges(self, map_params):
        n_roads, n_lanes_in, n_lanes_out, road_sep, road_angles, t_inner, t_outer, airline_dist = map_params
        def indices_with_lanes(n_lanes_lst):
            return [i for i in range(len(n_lanes_lst)) if n_lanes_lst[i] > 0]

        from_edges = []
        to_edges = []
        if self._task == 'l':
            for i in indices_with_lanes(n_lanes_in):
                from_edge = i
                to_edge = i - 1 if i >= 1 else n_roads - 1
                if n_lanes_out[to_edge] > 0:
                    from_edges.append(from_edge)
                    to_edges.append(to_edge)
        elif self._task == 'r':
            for i in indices_with_lanes(n_lanes_in):
                from_edge = i
                to_edge = i + 1 if i < n_roads - 1 else 0
                if n_lanes_out[to_edge] > 0:
                    from_edges.append(from_edge)
                    to_edges.append(to_edge)
        elif self._task == 's':  # Find two possible candidate in-out parring that results in a maximally straight crossing
            from_edge = None
            to_edges = None
            best_crossing_angle = 0
            for i in indices_with_lanes(n_lanes_in):
                for j in indices_with_lanes(n_lanes_out):
                    if i == j:
                        continue
                    from driver_dojo.common.utils import wrap_to_pi
                    crossing_angle = np.abs(np.radians(180) - wrap_to_pi(road_angles[i] - road_angles[j]))
                    if crossing_angle < best_crossing_angle:
                        best_crossing_angle = crossing_angle
                    from_edge = i
                    to_edge = j

            if from_edge is not None and to_edge is not None:
                from_edges = [from_edge]
                to_edges = [to_edge]
        else:
            raise ValueError("'IntersectionScenario' only supports 'r', 'l' or 's' as driving task specifications!")

        return from_edges, to_edges

    #def _sample_route(self):

    # def _sample_edge_priorities(self, rng: np.random.Generator, map_params, from_edge, to_edge):
    #     n_roads, n_lanes_in, n_lanes_out, road_sep, road_angles, t_inner, t_outer, airline_dist = map_params
    #
    #     edge_prios = np.ones((n_roads,))
    #     if self._crossing_style == 'rbl':
    #         return edge_prios
    #     elif self._crossing_style == 'minor':
    #         edge_prios[to_edge] = 2