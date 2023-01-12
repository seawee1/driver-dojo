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
            base_path,
            scenario_config,
            sim_config,
            net_seed,
            traffic_seed,
            task_seed,
            lock,
            generate=True,
    ):
        self._base_path = base_path
        self._sim_config = sim_config
        self._scenario_config = scenario_config
        self.net_seed: int = net_seed
        self.traffic_seed: int = traffic_seed
        self.task_seed: int = task_seed

        self.network_rng = np.random.default_rng(net_seed)
        self.traffic_rng = np.random.default_rng(traffic_seed)
        self.task_rng = np.random.default_rng(task_seed)

        self.rnd_token = None

        import random

        tasks = self._scenario_config.tasks  # Otherwise, with num_tasks set to 1, we would always sample the same task for every map
        random.Random(self.net_seed).shuffle(tasks)
        self._task = None if len(tasks) == 0 else self.task_rng.choice(tasks).lower()  # Draw the task
        self._map_params = None
        self.sumo_net = None
        self._lock = lock

        self.sumocfg_path = self._scenario_base_path + '.sumocfg'
        self.sumo_net_path = self._scenario_base_path + '.net.xml'
        self.sumo_route_path = self._scenario_config.route_path  #self._scenario_base_path + '.rou.xml'  # TODO: ScenarioConfig `route_path` and SumoEngine include
        self.xodr_path = self._scenario_base_path + '.xodr'
        self._sumo_vType_dist_path = self._scenario_base_path + '.add.xml'
        self._map_params_path = self._scenario_base_path + '.pkl'

        self.time_since_last_spawn = 0  # For BasicScenario.step()
        self.num_spawns = 0
        if generate:
            self._generate_init()

    def _generate_init(self):
        # import random
        # import string
        # str = ''
        # for i in range(10):
        #     str += random.choice(string.ascii_uppercase + string.digits)
        # self.rnd_token = str
        self.task_realisable = self.generate()

        traffic_low, traffic_high = self._scenario_config.traffic_scale
        self.traffic_scale = self.traffic_rng.uniform(traffic_low, traffic_high)
        self._route_edges = None

    def generate(self):
        import pickle
        # if self._lock is not None:
        #     self._lock.acquire()
        #tmp_base_path = self._scenario_base_path + self.rnd_token
        #if not os.path.isfile(self._map_params_path) or not os.path.isfile(self.sumo_net_path):
        while True:
            self._map_params = self.sample_map_parameters(self.network_rng)
            self.generate_map(self._map_params, self._scenario_base_path)

            try:
                self.sumo_net = sumolib.net.readNet(self.sumo_net_path, withInternal=True)  # Read the network for further processing
            except Exception:
                continue
            #
            # import shutil
            #if not os.path.isfile(self.sumo_net_path):
            write_sumocfg(self.sumocfg_path, self.sumo_net_path, self.sumo_route_path)  # TODO: copy scenario to this folder for static scenarios
            #     shutil.copy(tmp_base_path + '.sumocfg', self.sumocfg_path)

            #shutil.copy(tmp_base_path + '.net.xml', self.sumo_net_path)
            #os.remove(tmp_base_path + '.net.xml')

            with open(self._map_params_path, 'wb') as f:
                pickle.dump(self._map_params, f)
            #shutil.copy(tmp_base_path + '.pkl', self._map_params_path)

            #with open(self._map_params_path, 'wb') as f:
            #    pickle.dump(self._map_params, f)
            break

        if self._scenario_config.behavior_dist and not os.path.isfile(self._sumo_vType_dist_path):  # Initialize vType distribution
            self.create_vType_distribution(self._sumo_vType_dist_path)
            #shutil.copy(tmp_base_path + '.add.xml', self._sumo_vType_dist_path)  # DUNNO!!!!
            #self._sumo_vType_dist_path = tmp_base_path + '.add.xml'

        # if self._lock is not None:
        #     self._lock.release()

        if self._map_params is None:
            with open(self._map_params_path, 'rb') as f:
                self._map_params = pickle.load(f)

        if self.sumo_net is None:
            try:
                self.sumo_net = sumolib.net.readNet(self.sumo_net_path, withInternal=True)  # Read the network for further processing
            except:
                return False
        task_realisable = self.task_specifics()
        if not task_realisable:
            return False

        return True

    def sample_map_params(self):
        return

    def task_specifics(self):
        return

    @property
    def route(self):
        if self.sumo_route_path != '':
            self._route_edges = parse_route(self.sumo_route_path, self._sim_config.egoID)
        return self._route_edges  # TODO: Might still be None

    @property
    def sumo_add_path(self):
        if self._scenario_config.behavior_dist:
            if self._scenario_config.add_path != '':
                return self._scenario_config.add_path + ',' + self._sumo_vType_dist_path
            else:
                return self._sumo_vType_dist_path
        return self._scenario_config.add_path

    @property
    def name(self):
        # if self._task is not None:
        #     name += f'_{self._task}_{self.task_seed}'
        return f"{self._scenario_config.name.lower()}_{self.net_seed}_{self.traffic_seed}"

    @property
    def _scenario_base_path(self):
        if self.rnd_token is None:
            import random
            import string
            str = ''
            for i in range(10):
                str += random.choice(string.ascii_uppercase + string.digits)
            self.rnd_token = str
        return os.path.join(self._base_path, self.name + '_' + self.rnd_token)

    def initialize(self, traci):
        """Code that is run once the scenario is loaded into the SUMO engine.
        Use this to do traffic and ego spawning/routing in case this is not defined over a .rou.xml file.
        """
        ego_from_edge_id = ''
        if self._scenario_config.ego_init:
            ego_from_edge_id = self.init_ego(traci)

        if self._scenario_config.traffic_init:
            spread = self._scenario_config.traffic_init_spread * self.traffic_scale
            excludes_extra = [ego_from_edge_id] if self._scenario_config.traffic_init_edges_exclude_ego else None
            self.init_traffic(spread, traci, excludes_extra=excludes_extra)

    def step(self, traci):
        """Code that is run every environment step.
        Use this to do traffic spawning/routing in case this is not defined over a .rou.xml file.
        """
        if self._scenario_config.traffic_spawn:
            spawn_period = self._scenario_config.traffic_spawn_period * (1.0 / self.traffic_scale)
            num_spawn = int(self.time_since_last_spawn // spawn_period)
            if num_spawn > 0:
                self.time_since_last_spawn -= num_spawn * spawn_period
                for i in range(num_spawn):
                    veh_id = f'traffic_spawn_{self.num_spawns}'
                    route_id = f'traffic_spawn_route_{self.num_spawns}'
                    self.spawn_traffic(veh_id, route_id, traci)
                    self.num_spawns += 1
            self.time_since_last_spawn += self._sim_config.dt

    def create_vType_distribution(self, path):
        vType_command = [
            "python",
            self._scenario_config.vTypeDistribution_py_path,
            self._scenario_config.behavior_dist_path,
            "--output",
            path,
            "--name",
            "vehDist",
            "--seed",
            str(self.traffic_seed),
            "--size",
            str(self._scenario_config.behavior_dist_num),
        ]
        p = subprocess.Popen(vType_command)#, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        #p.wait()
        waiting_since = 0.0
        while p.poll() is None:
            if waiting_since > 10.0:
                self.task_realisable = False
                return
            import time
            time.sleep(1.0)
            waiting_since += 1.0

    def init_traffic(self, spread, traci, excludes_extra=None):
        if excludes_extra is None:
            excludes_extra = []

        init_edges = [x for x in self.traffic_init_edges if x not in excludes_extra]
        init_edges_lengths = [self.sumo_net.getEdge(x).getLength() for x in init_edges]
        num_per_edge = [int(round(x / spread)) for x in init_edges_lengths]

        cnt = 0
        traffic_vTypes = self.traffic_veh_types
        depart_pos, depart_lane, depart_speed = self._scenario_config.traffic_init_params
        for edge_id, num in zip(init_edges, num_per_edge):
            for i in range(num):
                veh_id = f'traffic_init_{cnt}'
                route_id = f'traffic_init_route_{cnt}'
                route_edge_ids = self.traffic_routing_strategy(edge_id)
                type_id = self.traffic_rng.choice(traffic_vTypes)
                add_vehicle(traci, veh_id, route_id, route_edge_ids, type_id, depart_pos, depart_lane, depart_speed)
                cnt += 1

    def init_ego(self, traci):
        veh_id = self._sim_config.egoID
        route_id = self._sim_config.routeID
        from_edge_id = self.traffic_rng.choice(self.ego_init_edges)
        route_edge_ids = self.ego_routing_strategy(from_edge_id)
        self._route_edges = route_edge_ids
        type_id = self.ego_veh_type
        depart_pos = self._scenario_config.ego_init_position
        depart_lane = self._scenario_config.ego_init_lane
        depart_speed = self._scenario_config.ego_init_speed
        add_vehicle(traci, veh_id, route_id, route_edge_ids, type_id, depart_pos, depart_lane, depart_speed)
        return from_edge_id

    def spawn_traffic(self, veh_id, route_id, traci):
        edge_id = self.traffic_rng.choice(self.traffic_spawn_edges)
        route_edge_ids = self.traffic_routing_strategy(edge_id)
        type_id = self.traffic_rng.choice(self.traffic_veh_types)
        depart_pos, depart_lane, depart_speed = self._scenario_config.traffic_spawn_params
        add_vehicle(traci, veh_id, route_id, route_edge_ids, type_id, depart_pos, depart_lane, depart_speed)

    @property
    def traffic_veh_types(self):
        """The vType of traffic participants. Will be `DEFAULT_VEHTYPE` in the default case."""
        if self._scenario_config.behavior_dist:
            vTypes = [f'vehDist{i}' for i in range(0, self._scenario_config.behavior_dist_num)]
        elif self._scenario_config.traffic_vTypes is not None:
            vTypes = self._scenario_config.traffic_vTypes
        else:
            vTypes = ['DEFAULT_VEHTYPE']
        return vTypes

    @property
    def ego_veh_type(self):
        return self._scenario_config.ego_vType if self._scenario_config.ego_vType is not None else 'DEFAULT_VEHTYPE'

    def traffic_routing_strategy(self, from_edge, rng=None) -> List[str]:
        """Given the starting `edge_id`, will return the edge id traffic should be routed towards.
        By default, will return a random outgoing edge that is reachable starting from given `from_edge_id`.
        """

        fringe_outgoing_edge_ids = [x.getID() for x in self.sumo_net.getEdges() if x.is_fringe() and len(x.getOutgoing()) == 0]
        possible_routes = []
        for edge_id in fringe_outgoing_edge_ids:
            route = self.sumo_net.getOptimalPath(self.sumo_net.getEdge(from_edge), self.sumo_net.getEdge(edge_id))
            if route[0] is not None:
                possible_routes.append([x.getID() for x in route[0]])

        if rng is None:
            return self.traffic_rng.choice(possible_routes)
        else:
            return rng.choice(possible_routes)

    def ego_routing_strategy(self, from_edge) -> List[str]:
        """Given the starting `edge_id`, will return the edge id the ego should be routed towards.
        By default, same as for traffic or as specified inside config tuple.
        """
        if self._scenario_config.ego_to_edges is not None:
            possible_routes = []
            for edge_id in self._scenario_config.ego_to_edges:
                route = self.sumo_net.getOptimalPath(self.sumo_net.getEdge(from_edge), self.sumo_net.getEdge(edge_id))
                if route[0] is not None:
                    possible_routes.append([x.getID() for x in route[0]])
            if len(possible_routes) == 0:
                raise ValueError("The ego route specification is not realisable!")
            return self.task_rng.choice(possible_routes)

        return self.traffic_routing_strategy(from_edge, self.task_rng)

    @property
    def ego_init_edges(self) -> Tuple[str]:
        """Returns a list of edge ids' the ego vehicle will be initialized on.
        By default, will spawn on fringe edges leading into the network, or as specified inside config.
        """
        if self._scenario_config.ego_from_edges is not None:
            return tuple(self._scenario_config.ego_from_edges)

        return tuple(str(y) for y in [x.getID() for x in self.sumo_net.getEdges(withInternal=False) if x.is_fringe() and len(x.getOutgoing()) > 0])

    @property
    def traffic_init_edges(self) -> Tuple[str]:
        """Returns a list of edge ids' traffic will be initialized on.
        By default, will spawn on all edges, or as specified inside config.
        """
        if self._scenario_config.traffic_init_edges is not None:  # config values have higher priority
            return tuple(self._scenario_config.traffic_init_edges)

        exclude_edge_ids = self._scenario_config.traffic_init_edges_exclude  # Take random edge excluding specified ones
        if exclude_edge_ids is None:
            exclude_edge_ids = []
        return tuple(str(y) for y in [x.getID() for x in self.sumo_net.getEdges(withInternal=False)] if y not in exclude_edge_ids)

    @property
    def traffic_spawn_edges(self) -> Tuple[str]:
        """Returns a list of edge ids' traffic will be spawned on.
        By default, will spawn on fringe edges leading into the network, or as specified inside config.
        """
        if self._scenario_config.traffic_spawn_edges is not None:
            return self._scenario_config.traffic_spawn_edges

        exclude_edge_ids = self._scenario_config.traffic_spawn_edges_exclude
        if exclude_edge_ids is None:
            exclude_edge_ids = []
        return tuple(str(y) for y in [x.getID() for x in self.sumo_net.getEdges(withInternal=False) if x.is_fringe() and len(x.getOutgoing()) > 0] if y not in exclude_edge_ids)

    def is_valid(self):
        return self._is_valid

    def generate_map(self):
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
