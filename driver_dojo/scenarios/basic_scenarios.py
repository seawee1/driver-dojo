import logging
import os
import subprocess
from abc import abstractmethod
import ntpath
from typing import Tuple, List, Optional

import numpy as np
import psutil
import sumolib.net

from driver_dojo.scenarios.utils import write_sumocfg, write_scenariogen_xodr


# TODO: Goaling
def add_vehicle(
        traci,
        veh_id,
        route_id,
        route_edge_ids: List[str],
        type_id,
        depart_pos,
        depart_lane,
        depart_speed
):
    # Add the route
    traci.route.add(route_id, route_edge_ids)
    logging.debug(f"Added route {route_id} with edge_ids {route_edge_ids}...")

    # Add the vehicle
    traci.vehicle.add(
        veh_id,
        route_id,
        type_id,
        departPos=depart_pos,
        departLane=depart_lane,
        departSpeed=depart_speed
    )
    logging.debug(f"Added vehicle {veh_id} of type {type_id} with route {route_id} with params ({depart_pos}, {depart_lane}, {depart_speed})...")


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
    return None


class BasicScenario:
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

        self.config = config
        self.net_seed: int = net_seed
        self.traffic_seed: int = traffic_seed
        self.sumocfg_path: str = sumocfg_path
        self.sumo_net_path: str = sumo_net_path
        self.sumo_route_path: str = sumo_route_path
        self._sumo_add_path: str = sumo_add_path  # The following two have to be combined in case both exist
        self._sumo_vType_path: str = sumo_vType_path
        self.xodr_path: str = xodr_path
        self.name = scenario_name

        self.time_since_last_spawn = 0  # For BasicScenario.step()
        self.num_spawns = 0

        self.network_rng = np.random.default_rng(self.net_seed)  # Initialize RNGs
        self.traffic_rng = np.random.default_rng(self.traffic_seed)

        if not os.path.isfile(sumo_net_path):  # In case we already generated this map at some point, pass
            self.generate()
        if not os.path.isfile(xodr_path):  # TODO: convert function
            self.convert()
        if not os.path.isfile(sumocfg_path):
            write_sumocfg(sumocfg_path, sumo_net_path, sumo_route_path)  # TODO: copy scenario to this folder for static scenarios

        if self.config.scenario.vType_rnd and not os.path.isfile(sumo_vType_path):  # Initialize vType distribution
            self.create_vType_distribution()

        traffic_low, traffic_high = self.config.scenario.traffic_scale
        self.traffic_scale = self.traffic_rng.uniform(traffic_low, traffic_high)

        self.sumo_net = sumolib.net.readNet(sumo_net_path, withInternal=True)  # Read the network for further processing
        self._route_edges = None

    @property
    def route(self):
        if self._route_edges is not None:
            return self._route_edges
        elif self.sumo_route_path is not None:
            self._route_edges = parse_route(self.sumo_route_path, self.config.simulation.egoID)
        return self._route_edges  # TODO: Might still be None

    @property
    def sumo_add_path(self):
        if self.config.scenario.vType_rnd:
            if self._sumo_add_path is not None:
                return f'{self._sumo_add_path},{self._sumo_vType_path}'
            return self._sumo_vType_path
        else:
            return self._sumo_add_path

    def initialize(self, traci):
        """Code that is run once the scenario is loaded into the SUMO engine.
        Use this to do traffic and ego spawning/routing in case this is not defined over a .rou.xml file.
        """
        ego_from_edge_id = ''
        if self.config.scenario.ego_init:
            ego_from_edge_id = self.init_ego(traci)

        if self.config.scenario.traffic_init:
            spread = self.config.scenario.traffic_init_spread * self.traffic_scale
            excludes_extra = [ego_from_edge_id] if self.config.scenario.traffic_init_edges_exclude_ego else None
            self.init_traffic(spread, traci, excludes_extra=excludes_extra)

    def step(self, traci):
        """Code that is run every environment step.
        Use this to do traffic spawning/routing in case this is not defined over a .rou.xml file.
        """
        if self.config.scenario.traffic_spawn:
            spawn_period = self.config.scenario.traffic_spawn_period * (1.0 / self.traffic_scale)
            num_spawn = int(self.time_since_last_spawn // spawn_period)
            if num_spawn > 0:
                self.time_since_last_spawn -= num_spawn * spawn_period
                for i in range(num_spawn):
                    veh_id = f'traffic_spawn_{self.num_spawns}'
                    route_id = f'traffic_spawn_route_{self.num_spawns}'
                    self.spawn_traffic(veh_id, route_id, traci)
                    self.num_spawns += 1
            self.time_since_last_spawn += self.config.simulation.dt

    def create_vType_distribution(self):
        vType_command = [
            "python",
            self.config.scenario.vType_rnd_path,
            self.config.scenario.vType_rnd_config_path,
            "--output",
            self.sumo_add_path,
            "--name",
            "vehDist",
            "--seed",
            str(self.traffic_seed),
            "--size",
            str(self.config.scenario.vType_rnd_num),
        ]
        p = subprocess.Popen(
            vType_command, stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )
        p.wait()

    def init_traffic(self, spread, traci, excludes_extra=None):
        if excludes_extra is None:
            excludes_extra = []

        init_edges = [x for x in self.traffic_init_edges if x not in excludes_extra]
        init_edges_lengths = [self.sumo_net.getEdge(x).getLength() for x in init_edges]
        num_per_edge = [int(round(x / spread)) for x in init_edges_lengths]

        cnt = 0
        traffic_vTypes = self.traffic_veh_types
        depart_pos, depart_lane, depart_speed = self.config.scenario.traffic_init_params
        for edge_id, num in zip(init_edges, num_per_edge):
            for i in range(num):
                veh_id = f'traffic_init_{cnt}'
                route_id = f'traffic_init_route_{cnt}'
                route_edge_ids = self.traffic_routing_strategy(edge_id)
                type_id = self.traffic_rng.choice(traffic_vTypes)
                add_vehicle(traci, veh_id, route_id, route_edge_ids, type_id, depart_pos, depart_lane, depart_speed)
                cnt += 1

    def init_ego(self, traci):
        veh_id = self.config.simulation.egoID
        route_id = self.config.simulation.routeID
        from_edge_id = self.traffic_rng.choice(self.ego_init_edges)
        route_edge_ids = self.ego_routing_strategy(from_edge_id)
        self._route_edges = route_edge_ids
        type_id = self.ego_veh_type
        depart_pos = self.config.scenario.ego_init_position
        depart_lane = self.config.scenario.ego_init_lane
        depart_speed = self.config.scenario.ego_init_speed
        add_vehicle(traci, veh_id, route_id, route_edge_ids, type_id, depart_pos, depart_lane, depart_speed)
        return from_edge_id

    def spawn_traffic(self, veh_id, route_id, traci):
        edge_id = self.traffic_rng.choice(self.traffic_spawn_edges)
        route_edge_ids = self.traffic_routing_strategy(edge_id)
        type_id = self.traffic_rng.choice(self.traffic_veh_types)
        depart_pos, depart_lane, depart_speed = self.config.scenario.traffic_spawn_params
        add_vehicle(traci, veh_id, route_id, route_edge_ids, type_id, depart_pos, depart_lane, depart_speed)

    @property
    def traffic_veh_types(self):
        """The vType of traffic participants. Will be `DEFAULT_VEHTYPE` in the default case."""
        if self.config.scenario.vType_rnd:
            vTypes = [f'vehDist{i}' for i in range(0, self.config.scenario.vType_rnd_num)]
        elif self.config.scenario.traffic_vTypes is not None:
            vTypes = self.config.scenario.traffic_vTypes
        else:
            vTypes = ['DEFAULT_VEHTYPE']
        return vTypes

    @property
    def ego_veh_type(self):
        return self.config.scenario.ego_vType if self.config.scenario.ego_vType is not None else 'DEFAULT_VEHTYPE'

    def traffic_routing_strategy(self, from_edge) -> List[str]:
        """Given the starting `edge_id`, will return the edge id traffic should be routed towards.
        By default, will return a random outgoing edge that is reachable starting from given `from_edge_id`.
        """

        fringe_outgoing_edge_ids = [x.getID() for x in self.sumo_net.getEdges() if x.is_fringe() and len(x.getOutgoing()) == 0]
        possible_routes = []
        for edge_id in fringe_outgoing_edge_ids:
            route = self.sumo_net.getOptimalPath(self.sumo_net.getEdge(from_edge), self.sumo_net.getEdge(edge_id))
            if route[0] is not None:
                possible_routes.append([x.getID() for x in route[0]])

        return self.traffic_rng.choice(possible_routes)

    def ego_routing_strategy(self, from_edge) -> List[str]:
        """Given the starting `edge_id`, will return the edge id ego should be routed towards.
        By default, same as for traffic or as specified inside config tuple.
        """
        if self.config.scenario.ego_to_edges is not None:
            possible_routes = []
            for edge_id in self.config.scenario.ego_to_edges:
                route = self.sumo_net.getOptimalPath(self.sumo_net.getEdge(from_edge), self.sumo_net.getEdge(edge_id))
                if route[0] is not None:
                    possible_routes.append([x.getID() for x in route[0]])
            if len(possible_routes) == 0:
                raise ValueError("The ego route specification is not realisable!")
            return self.traffic_rng.choice(possible_routes)

        return self.traffic_routing_strategy(from_edge)

    @property
    def ego_init_edges(self) -> Tuple[str]:
        """Returns a list of edge ids' the ego vehicle will be initialized on.
        By default, will spawn on fringe edges leading into the network, or as specified inside config.
        """
        if self.config.scenario.ego_from_edges is not None:
            return tuple(self.config.scenario.ego_from_edges)

        return tuple(str(y) for y in [x.getID() for x in self.sumo_net.getEdges(withInternal=False) if x.is_fringe() and len(x.getOutgoing()) > 0])

    @property
    def traffic_init_edges(self) -> Tuple[str]:
        """Returns a list of edge ids' traffic will be initialized on.
        By default, will spawn on all edges, or as specified inside config.
        """
        if self.config.scenario.traffic_init_edges is not None:  # config values have higher priority
            return tuple(self.config.scenario.traffic_init_edges)

        exclude_edge_ids = self.config.scenario.traffic_init_edges_exclude  # Take random edge excluding specified ones
        if exclude_edge_ids is None:
            exclude_edge_ids = []
        return tuple(str(y) for y in [x.getID() for x in self.sumo_net.getEdges(withInternal=False)] if y not in exclude_edge_ids)

    @property
    def traffic_spawn_edges(self) -> Tuple[str]:
        """Returns a list of edge ids' traffic will be spawned on.
        By default, will spawn on fringe edges leading into the network, or as specified inside config.
        """
        if self.config.scenario.traffic_spawn_edges is not None:
            return self.config.traffic_spawn_edges

        exclude_edge_ids = self.config.scenario.traffic_spawn_edges_exclude
        if exclude_edge_ids is None:
            exclude_edge_ids = []
        return tuple(str(y) for y in [x.getID() for x in self.sumo_net.getEdges(withInternal=False) if x.is_fringe() and len(x.getOutgoing()) > 0] if y not in exclude_edge_ids)

    def generate(self):
        return

    def convert(self):
        return

# class ScenarioGenScenario(BasicScenario):
#     @property
#     def _netconvert_to_sumo_command(self):
#         return [
#             "netconvert",
#             "--verbose",
#             'true',
#             "--junctions.internal-link-detail",
#             "10",
#             "--junctions.corner-detail",
#             "20",
#             "--opendrive.import-all-lanes",
#             "false",
#             "--opendrive.curve-resolution",
#             "1.0",
#             "--opendrive.internal-shapes",
#             "false",
#             "--opendrive.lane-shapes",
#             "false",
#             "--geometry.max-grade.fix",
#             "true",
#             "--plain.extend-edge-shape",
#             "false",
#             "--no-turnarounds",
#             "true",
#             "--default.junctions.keep-clear",
#             "false",
#             "--opendrive.advance-stopline",
#             "15",  # For more natural intersections
#             "--keep-nodes-unregulated",
#             "false",
#             "--offset.disable-normalization",
#             "false",
#             "--geometry.min-radius.fix",
#             "true",
#             "--check-lane-foes.all",
#             "true",
#         ]
#
#     def _netconvert_to_sumo_command_with_paths(self, sumo_net_path, xodr_path):
#         netconvert_command = self._netconvert_to_sumo_command
#         netconvert_command += [
#             "--opendrive",
#             xodr_path,
#             "--output-file",
#             sumo_net_path
#         ]
#         return netconvert_command
#
#     def generate_and_convert(self, rng, sumo_net_path, xodr_path):
#         zero_cloth = True
#         while zero_cloth:  # Sometimes we can have faulty scenarios (indicated by `zero_cloth`)
#             parameters = self.sample_map_parameters(rng)
#             roads, junctions, zero_cloth = self._generate_map(parameters)
#
#         write_scenariogen_xodr(roads, junctions, xodr_path)
#         netconvert_command = self._netconvert_to_sumo_command_with_paths(sumo_net_path, xodr_path)
#         p = subprocess.Popen(
#             netconvert_command, stderr=subprocess.PIPE, stdout=subprocess.PIPE
#         )
#
#         def kill(proc_pid):
#             process = psutil.Process(proc_pid)
#             for proc in process.children(recursive=True):
#                 proc.kill()
#             process.kill()
#
#         try:  # Sometimes netconvert can't convert the scenario to .net.xml, in which case we generate a new map with different parameters
#             p.wait(2.0)
#         except subprocess.TimeoutExpired:
#             try:
#                 kill(p.pid)
#             except psutil.NoSuchProcess:
#                 pass
#             p.wait()
#             self.generate_and_convert(rng, sumo_net_path, xodr_path)
#
#     @abstractmethod
#     def _generate_map(self, kwargs) -> Tuple:
#         """`generate_map_and_write` will use `_generate_map` internally."""
#         return
#
#     @abstractmethod
#     def sample_map_parameters(self, rng) -> Tuple:
#         """In case the scenario is parameterized, define parameter sampling here.
#         Should return a tuple of parameters.
#         """
#         return
#
#
# class SumolibScenario(BasicScenario):
#     def generate_and_convert(self, rng, sumo_net_path, xodr_path):
#         # TODO
#         return
#
#     @abstractmethod
#     def _generate_map(self, kwargs):
#         """`generate_map_and_write` will use `_generate_map` internally."""
#         return
#
#     @abstractmethod
#     def sample_map_parameters(self, rng):
#         """In case the scenario is parameterized, define parameter sampling here.
#         Should return a tuple of parameters.
#         """
#         return
