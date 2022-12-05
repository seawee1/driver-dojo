import subprocess
from typing import List, Dict
import numpy as np
from pyclothoids import Clothoid

from driver_dojo.scenarios.basic_scenarios import BasicScenario
from driver_dojo.scenarios.plainxml import Edge, Node, Connection, write_plain_xml


class IntersectionScenario(BasicScenario):
    def __init__(
            self,
            config,
            net_seed,
            traffic_seed,
            sumocfg_path,
            sumo_net_path,
            sumo_route_path,
            sumo_add_path,
            sumo_vType_path,
            xodr_path,
            scenario_name,
    ):
        self._intersection_size = scenario_name.split('_')[1].lower()
        assert self._intersection_size in ['s', 'l']
        self._intersection_type = scenario_name.split('_')[2].lower()
        assert self._intersection_type in ['major', 'minor', 'rbl']

        if self._intersection_type == 'major':
            self._target_conn_state = 'M'
        elif self._intersection_type == 'minor':
            self._target_conn_state = 'm'
        else:
            self._target_conn_state = None

        self._node_name: str = 'main'
        self._shape_step: float = 1.0
        self._nodes: Dict[str, Node] = {}
        self._edges: Dict[str, Edge] = {}
        self._xml_base_path: str = sumo_net_path[:-8]  # Drop the '.net.xml' file ending

        super().__init__(
            config,
            net_seed,
            traffic_seed,
            sumocfg_path,
            sumo_net_path,
            sumo_route_path,
            sumo_add_path,
            sumo_vType_path,
            xodr_path,
            scenario_name,
        )

    @property
    def _valid_connections(self):
        conns = []
        for conn in self.sumo_net.getNode(self._node_name).getConnections():  # Get all junction connections
            if conn.getFrom().getFunction() != 'internal' and conn.getTo().getFunction() != 'internal':
                conns.append(conn)
        return conns

    @property
    def ego_init_edges(self):
        ego_init_edges = []
        for conn in self._valid_connections:  # Get all junction connections
            if self._target_conn_state is None or conn.getState() == self._target_conn_state:
                ego_init_edges.append(conn.getFrom().getID())
        assert len(ego_init_edges) > 0, f'{self.net_seed}, {self.traffic_seed}'
        return ego_init_edges

    def ego_routing_strategy(self, from_edge: str) -> List[str]:
        ego_target_edges = []
        for conn in self._valid_connections:  # Get all junction connections
            if conn.getFrom().getID() == from_edge and (conn.getState() == self._target_conn_state or self._target_conn_state is None):
                ego_target_edges.append(conn.getTo().getID())

        to_edge = self.traffic_rng.choice(ego_target_edges)
        route = self.sumo_net.getOptimalPath(self.sumo_net.getEdge(from_edge), self.sumo_net.getEdge(to_edge))
        assert route[0] is not None
        return [x.getID() for x in route[0]]

    def generate(self):
        params = self.sample_map_parameters(self.network_rng)
        self._generate_map(params)

    def _netconvert_cmd(self, nodes_xml_path, edges_xml_path):
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
            self.sumo_net_path,
            '--check-lane-foes.all',
            'true'
        ]

    def _generate_map(self, params):
        n_roads, n_lanes_in, n_lanes_out, road_sep, road_angles, t_inner, t_outer, airline_dist, priorities = params

        node_type = 'right_before_left' if self._intersection_type == 'rbl' else 'priority'
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
                self._edges[f'{i}_in'] = Edge(f'{i}_in', str(i), self._node_name, numLanes=n_lanes_in[i], shape=shape, priority=priorities[i])

            if n_lanes_out[i] > 0:
                if road_sep[i] > 0.0:
                    orth_angle = road_angles[i] - np.pi / 2
                    x_off, y_off = np.cos(orth_angle) * road_sep[i], np.sin(orth_angle) * road_sep[i]
                    cloth = cloth.Translate(x_off, y_off)
                shape = cloth.SampleXY(num_shape_samples)
                self._edges[f'{i}_out'] = Edge(f'{i}_out', self._node_name, str(i), numLanes=n_lanes_out[i], shape=shape, priority=priorities[i])

        nodes_xml_path, edges_xml_path, conns_xml_path = write_plain_xml(self._xml_base_path, self._nodes.values(), self._edges.values(), conns=None)
        p = subprocess.Popen(
            self._netconvert_cmd(nodes_xml_path, edges_xml_path)#, stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )
        p.wait()

    def sample_map_parameters(self, rng: np.random.Generator):
        if self._intersection_size == 'l':  # TODO: refactor
            n_roads = rng.choice([3, 4, 5])
            n_lanes_in = [rng.choice([0, 1, 2, 3], p=[0.1, 0.3, 0.3, 0.3]) for i in range(n_roads)]
            n_lanes_out = [rng.choice([0, 1, 2, 3], p=[0.1, 0.3, 0.3, 0.3]) for i in range(n_roads)]
            for i, (n_in, n_out) in enumerate(zip(n_lanes_in, n_lanes_out)):  # Enfore `n_roads`
                if n_in == 0 and n_out == 0:
                    out = bool(rng.integers(0, 2))
                    if out:
                        n_lanes_out[i] = rng.choice([1, 2])
                    else:
                        n_lanes_in[i] = rng.choice([1, 2])
            if sum(n_lanes_in) == 0:  # Enforce at least one `in` road and one `out`
                n_lanes_in[rng.integers(0, n_roads)] = rng.choice([1, 2, 3])
            if sum(n_lanes_out) == 0:
                n_lanes_out[rng.integers(0, n_roads)] = rng.choice([1, 2, 3])
        elif self._intersection_size == 's':
            n_roads = rng.choice([3, 4])
            n_lanes_in = [rng.choice([0, 1, 2], p=[0.1, 0.45, 0.45]) for i in range(n_roads)]
            n_lanes_out = [rng.choice([0, 1, 2], p=[0.1, 0.45, 0.45]) for i in range(n_roads)]
            for i, (n_in, n_out) in enumerate(zip(n_lanes_in, n_lanes_out)):
                if n_in == 0 and n_out == 0:
                    out = bool(rng.integers(0, 2))
                    if out:
                        n_lanes_out[i] = rng.choice([1, 2])
                    else:
                        n_lanes_in[i] = rng.choice([1, 2])
            if sum(n_lanes_in) == 0:
                n_lanes_in[rng.integers(0, n_roads)] = rng.choice([1, 2])
            if sum(n_lanes_out) == 0:
                n_lanes_out[rng.integers(0, n_roads)] = rng.choice([1, 2])
        else:
            raise ValueError
        road_sep = [rng.uniform(0.2, 2.0) if rng.uniform(0.0, 1.0) >= 0.5 else 0.0 for i in range(n_roads)]
        angle_var = 0.8 * (365.0 / n_roads / 2)
        road_angles = [
            np.radians(i * 365.0 / n_roads + rng.integers(-angle_var, angle_var)) for i in range(n_roads)
        ]
        t_inner = [rng.uniform(road_angle - np.pi / 16, road_angle + np.pi / 16) for road_angle in road_angles]
        t_outer = [rng.uniform(road_angle - np.pi / 4, road_angle + np.pi / 4) for road_angle in road_angles]
        airline_dist = [rng.integers(50.0, 100.0) for i in range(n_roads)]

        priorities = np.ones((n_roads,))  # Make two roads higher priority
        if self._intersection_type != 'rbl':
            for i in range(2):
                priorities[rng.choice(np.arange(0, n_roads)[priorities == 1])] = 2

        return n_roads, n_lanes_in, n_lanes_out, road_sep, road_angles, t_inner, t_outer, airline_dist, priorities
