import os

from os.path import join as pjoin
import gym
import sumolib.geomhelper
import traci
import logging
import logging.config
from traci.exceptions import TraCIException
import tempfile
import string
import gym.utils.seeding as seeding
import xml.etree.cElementTree as ET
import numpy as np
from omegaconf import OmegaConf

from driver_dojo.actions import DirectActions, SemanticActions
from driver_dojo.common.utils import polygon_circle_shape
from driver_dojo.navigation import SUMOMap, SubGoalManager, WaypointManager
from driver_dojo.vehicle import TUMVehicle, TPSVehicle
from driver_dojo.variation.scenario_generator import ScenarioGenerator
from driver_dojo.observer import (
    EgoVehicleObserver,
    MultiObserver,
    RadiusVehicleObserver,
    WaypointObserver,
)
from driver_dojo.observer import (
    BirdsEyeObserver,
    AvailableOptionsObserver,
    SubGoalObserver,
    RoadShapeObserver,
)
from driver_dojo.common import TrafficManager
from driver_dojo.variation import TrafficRandomizer
import driver_dojo.common.collision_detection as collision_detection
import driver_dojo.common.runtime_vars as runtime_vars
from driver_dojo.core import SUMOEngine
from driver_dojo.core.config import *
from driver_dojo.core.types import ActionSpace


# from driver_dojo.render.renderer import Renderer


def _init_sumo_paths():
    # We will save server-specific files here
    temp_path = tempfile.mkdtemp(prefix="sumo_")

    if (
            runtime_vars.config["variation"]["network"]
            or runtime_vars.config["variation"]["network_netgenerate"]
    ):
        runtime_vars.config["simulation"]["net_path"] = pjoin(
            temp_path, f"{runtime_vars.sumo_label}.net.xml"
        )
    if runtime_vars.config["variation"]["traffic_routing_randomTrips"]:
        runtime_vars.config["simulation"]["route_path"] = pjoin(
            temp_path, f"{runtime_vars.sumo_label}.rou.xml"
        )
    if runtime_vars.config["variation"]["traffic_vTypes"]:
        if runtime_vars.config["simulation"]["add_path"] is None:
            runtime_vars.config["simulation"]["add_path"] = ""
        else:
            runtime_vars.config["simulation"]["add_path"] += ","
        runtime_vars.config["simulation"]["add_path"] += pjoin(
            temp_path, f"{runtime_vars.sumo_label}.add.xml"
        )

    # Create {label}.sumocfg
    runtime_vars.sumocfg_path = pjoin(
        temp_path, f"{runtime_vars.sumo_label}.sumocfg"
    )
    root = ET.Element("configuration")
    inp = ET.SubElement(root, "input")
    ET.SubElement(
        inp, "net-file", value=runtime_vars.config["simulation"]["net_path"]
    )
    if runtime_vars.config["simulation"]["route_path"]:
        ET.SubElement(
            inp, "route-files", value=runtime_vars.config["simulation"]["route_path"]
        )
    tree = ET.ElementTree(root)
    tree.write(runtime_vars.sumocfg_path)

    runtime_vars.temp_path = temp_path


def _init_seeding():
    # We initialize the np_random RandomGenerator with the specified seed
    runtime_vars.np_random, _ = seeding.np_random(
        runtime_vars.config["simulation"]["seed"]
    )

    # In case we use a list of seeds for the scenarios and/or traffic, create the seed lists.
    if (
            runtime_vars.config["simulation"]["seed_num_maps"]
            and not runtime_vars.seed_list_maps
    ):
        runtime_vars.seed_list_maps = [
            runtime_vars.np_random.randint(13371137)
            for _ in range(runtime_vars.config["simulation"]["seed_num_maps"])
        ]
    if (
            runtime_vars.config["simulation"]["seed_num_traffic"]
            and not runtime_vars.seed_list_traffic
    ):
        runtime_vars.seed_list_traffic = [
            runtime_vars.np_random.randint(13371137)
            for _ in range(runtime_vars.config["simulation"]["seed_num_traffic"])
        ]


def _seed(seed=None):
    if not seed:
        seed = runtime_vars.np_random.randint(13371337)

    runtime_vars.np_random, runtime_vars.level_seed = seeding.np_random(seed)

    # Map generation seeding
    if runtime_vars.seed_list_maps:
        seed_index = runtime_vars.np_random.randint(
            0, len(runtime_vars.seed_list_maps)
        )
        (
            runtime_vars.np_random_maps,
            runtime_vars.maps_seed,
        ) = seeding.np_random(runtime_vars.seed_list_maps[seed_index])

    else:
        rnd_seed = runtime_vars.np_random.randint(13371337)
        (
            runtime_vars.np_random_maps,
            runtime_vars.maps_seed,
        ) = seeding.np_random(rnd_seed)

    # Traffic seeding
    if runtime_vars.seed_list_traffic:
        seed_index = runtime_vars.np_random.randint(
            0, len(runtime_vars.seed_list_traffic)
        )
        (
            runtime_vars.np_random_traffic,
            runtime_vars.traffic_seed,
        ) = seeding.np_random(runtime_vars.seed_list_traffic[seed_index])
    else:
        rnd_seed = runtime_vars.np_random.randint(13371337)
        (
            runtime_vars.np_random_traffic,
            runtime_vars.traffic_seed,
        ) = seeding.np_random(rnd_seed)


class DriverDojoEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, config=None, _config=None):
        # Initialize config
        # 'config' is for external configuration, whereas '_config' is used for internal configuration.
        # config defaults defined inside config.py < config_ < config
        #   => You can overwrite everything externally with specifications through the 'config' parameter.
        self.config = Config()
        if _config:
            self.config = OmegaConf.merge(self.config, _config)
        if config:
            self.config = OmegaConf.merge(self.config, config)
        runtime_vars.config = self.config

        # Set logging level
        if runtime_vars.config.simulation.info:
            logging.basicConfig(level=logging.INFO)
        else:
            logging.basicConfig(level=logging.WARNING)
            # logger = logging.getLogger()
            # logger.propagate = False
            # logger.disable = True

        # Initial seeding hook
        _init_seeding()
        _seed()

        # Set SUMO server label
        literals = string.ascii_uppercase + string.digits
        runtime_vars.sumo_label = "".join(
            [literals[np.random.randint(0, len(literals))] for _ in range(20)]
        )

        # Set simulation file paths, create tmp folder, initialize .sumocfg file
        _init_sumo_paths()

        # Create core objects
        runtime_vars.sumo_engine = SUMOEngine()
        runtime_vars.sumo_map = SUMOMap()
        runtime_vars.traffic_manager = TrafficManager()
        runtime_vars.scenario_generator = ScenarioGenerator()

        # Create the scenario, wait until created.
        runtime_vars.scenario_generator.step()
        runtime_vars.scenario_generator.join()
        # Initialize the engine
        runtime_vars.traci = runtime_vars.sumo_engine.init_engine()

        runtime_vars.sumo_engine.simulationStep()

        # Initialize traffic (if desired -> else: Use provided rou.xml files)
        if runtime_vars.config.simulation.init_traffic:
            self._init_traffic()
        # Initialize ego (if desired -> else: search for vehicle 'egoID' and route it along 'route_ego'
        if runtime_vars.config.simulation.init_ego:
            self._init_ego()

        # We have to do a first initialisation step
        runtime_vars.sumo_engine.simulationStep()

        # Start the next generation in the background.
        runtime_vars.scenario_generator.step()

        # Initialize core environment objects
        runtime_vars.sumo_map = SUMOMap()
        runtime_vars.traffic_manager = TrafficManager()
        runtime_vars.vehicle = self._create_vehicle()

        # The renderer TODO
        # self.renderer = Renderer()

        # Initialize observation_space and action_space
        # TODO: remove @property decorators. They're a little to artsy here.
        self._observer = None
        self._observation_space = None
        self._observation_space = (
            self.observation_space
        )  # We do this to trigger initialization of MultiObserver object
        self._actions = None
        self._action_space = None
        self._action_space = self.action_space  # Same here

        # Helper variables
        self._last_sub_goals = None
        self.stepped = False
        self.time_step = None
        self.standing_still_for = None
        self._rendered_waypointIDs = None
        self._rendered_goalIDs = None
        self._config = None
        self._time_since_last_insert = 0.0
        runtime_vars.episode_count = 0

        logging.info(f"Working in temporary directory {runtime_vars.temp_path}")

        logging.info("==============================================")
        logging.info("=================CONFIGURATION================")
        logging.info("==============================================")
        for cat, d in runtime_vars.config.items():
            logging.info(cat)
            for c, val in d.items():
                logging.info(f"    {c}: {val}")
        logging.info("==============================================")

    def reset(self, seed=None):
        # self.renderer.reset()

        # Re-init SUMO engine
        # If we didn't call step() since the last reset() (and seed didn't change) we can save time and skip this.
        # if self.stepped or seed or True:
        # Seeding
        _seed(seed)

        runtime_vars.episode_count += 1
        runtime_vars.scenario_generator.join()
        runtime_vars.sumo_engine.init_engine()

        if runtime_vars.sumo_engine.simulationStep(): return self.reset()

        # Initialize traffic (if desired -> else: Use provided rou.xml files)
        if runtime_vars.config.simulation.init_traffic:
            self._init_traffic()
        # Initialize ego (if desired -> else: search for vehicle 'egoID' and route it along 'route_ego'
        if runtime_vars.config.simulation.init_ego:
            self._init_ego()

        # Do initial simulation step
        if runtime_vars.sumo_engine.simulationStep(): return self.reset()
        runtime_vars.scenario_generator.step()

        # Rebuild lane graph for routing and sub-goaling
        runtime_vars.sumo_map.build_lane_graph()

        # Track ego vehicle in GUI if requested
        if (
                runtime_vars.config["simulation"]["render"]
                and runtime_vars.config["simulation"]["render_track_ego"]
        ):
            runtime_vars.traci.gui.trackVehicle(
                traci.gui.DEFAULT_VIEW, runtime_vars.config["simulation"]["egoID"]
            )
            runtime_vars.traci.gui.setZoom(traci.gui.DEFAULT_VIEW, 200.0)

        # Resets
        runtime_vars.traffic_manager.reset()
        runtime_vars.traffic_manager.step()  # We need to step one time to initialize storage
        runtime_vars.vehicle.reset()
        self._observer.reset()

        # Initialize the environment by letting it run for 'init_time' seconds
        num_init_steps = int(
            runtime_vars.config["simulation"]["init_time"]
            / runtime_vars.config["simulation"]["dt"]
        )
        for i in range(num_init_steps):
            # Insert new traffic
            if runtime_vars.config.variation.traffic_routing:
                self._insert_traffic()

            if runtime_vars.sumo_engine.simulationStep(): return self.reset()
            runtime_vars.vehicle.reset()
            runtime_vars.traffic_manager.step()

        # Reset the state held by the BaseActions sibling
        self._actions.reset()

        # Other variables
        self.time_step = 0
        runtime_vars.time_step = self.time_step
        self.standing_still_for = 0
        self._rendered_waypointIDs = []
        self._rendered_goalIDs = []
        self._time_since_last_insert = 0.0
        self.stepped = False

        obs = self._observer.step()
        return obs

    def _dummy_return(self):
        obs = np.zeros(self.action_space.shape, dtype=np.float32)
        rew = 0.0
        done = True
        info = dict(
            cost=0,  # Many safety-related methods need this
            level_seed=runtime_vars.level_seed,  # Methods like PLR make use of this
            time_step=runtime_vars.time_step,
            reached_goal=False,
            collision=False,  # For stats
            timeout=False,
            timeout_standing_still=False,
            off_route=False,
        )
        return obs, rew, done, info

    def step(self, action):
        # TODO
        # self.renderer.step()

        self.stepped = True
        self._last_sub_goals = runtime_vars.vehicle.sub_goals

        self._action_loop(action)

        obs = self._observer.step()
        reward, done, info = self._reward_done_info

        # Record images/videos
        if runtime_vars.config.simulation.render_record:
            self._record()

        return obs, reward, done, info

    def render(self, mode="human"):
        # sumo-gui is doing our rendering
        return

    def close(self):
        # Close the engine
        runtime_vars.sumo_engine.close_engine()

    def _calc_target_edge(self, startID, edgeIDs, strat):
        # TODO: I don't like this whole . Can definitely be implemented more nicely.
        # Find out proper goal edge
        if strat == "min-or-0":  # Strategy for Highway Scenarios
            if "-" in startID:
                target_edge = str(min(edgeIDs))
            else:
                target_edge = str(0)
        elif strat == "highway-exit":
            if "-" in startID:
                if startID == "-3":
                    target_edge = "-3"
                else:
                    target_edge = runtime_vars.np_random_traffic.choice(["-3", "-2"])
                # If this is the case, we have a splitting road that is converted into "0$0", "0$1" and "0#2"
            else:
                target_edge = "0#0"
        elif strat == "same":
            target_edge = None
        elif strat == "intersection":
            if "-" in startID:
                return None
            num_edges = max(edgeIDs) + 1
            target_edges = [
                -x for x in range(0, num_edges) if abs(int(startID)) != abs(x)
            ]
            target_edge = runtime_vars.np_random_traffic.choice(target_edges)
            if target_edge == 0:
                target_edge = str("-0")
            else:
                target_edge = str(target_edge)
        elif strat == "roundabout":
            if "out" in startID:
                return None
            edgeIDs = [id for id in edgeIDs if "out" in id]
            target_edge = runtime_vars.np_random_traffic.choice(edgeIDs)
        else:
            raise NotImplementedError

        if target_edge == startID:
            return None
        return target_edge

    def _init_traffic(self):
        # Read in from config
        dest_strat = runtime_vars.config.simulation.init_traffic_goal_strategy
        exclude_edgeIDs = (
            runtime_vars.config.simulation.init_traffic_exclude_edges
            if runtime_vars.config.simulation.init_traffic_exclude_edges
            else []
        )
        traffic_spread = runtime_vars.config.simulation.init_traffic_spread

        edges = [
            edge
            for edge in runtime_vars.net.getEdges(withInternal=False)
            if edge.getLength() > 1.0
        ]
        edgeIDs = [edge.getID() for edge in edges]
        edges_num_vehicles = dict()
        street_area = 0.0
        # Find out total area of street network
        for edge in edges:
            area = edge.getLength() * len(edge.getLanes())
            street_area += area
            edges_num_vehicles[edge.getID()] = area

        # Define how many vehicles to spawn in total
        num_vehicles = int(round(street_area / traffic_spread))
        for edge in edges:
            if edge.getID() in exclude_edgeIDs:
                continue
            edges_num_vehicles[edge.getID()] = int(
                round(edges_num_vehicles[edge.getID()] / street_area * num_vehicles)
            )

        # We have to remove those. These are edges part of a merge/split.
        # TODO: refactor
        if runtime_vars.config.simulation.init_ego_strategy != "roundabout":
            edgeIDs_goal = [int(edgeID) for edgeID in edgeIDs if "#" not in edgeID]
        else:
            edgeIDs_goal = edgeIDs

        for edgeID, num in edges_num_vehicles.items():
            if edgeID in exclude_edgeIDs:
                continue

            target_edge = self._calc_target_edge(
                edgeID,
                edgeIDs_goal,
                runtime_vars.config.simulation.init_traffic_goal_strategy,
            )

            for i in range(num):
                routeID = f"route_trafficInit_{edgeID}_{i}"
                vehID = f"trafficInit_{edgeID}_{i}"
                route_edges = [edgeID]
                if (
                        target_edge
                ):  # If None, we only drive the current edge from start to finish
                    route_edges += [target_edge]
                runtime_vars.traci.route.add(routeID, route_edges)

                typeID = "DEFAULT_VEHTYPE" if not runtime_vars.config.variation.traffic_vTypes else f"vehDist{runtime_vars.np_random_maps.randint(0, runtime_vars.config.variation.vTypeDistribution_num)}"
                runtime_vars.traci.vehicle.add(
                    vehID,
                    routeID,
                    typeID=typeID,
                    departPos="random_free",
                    departLane="best",
                    departSpeed="random",
                )

                logging.debug(
                    f"Traffic init: {vehID} on edge {edgeID} towards {route_edges[-1]}."
                )

    def _init_ego(self):
        egoID = runtime_vars.config.simulation.egoID
        routeID = runtime_vars.config.simulation.routeID
        start_edgeID = runtime_vars.config.simulation.init_ego_start_edge
        goal_edgeID = runtime_vars.config.simulation.init_ego_goal_edge

        start_edge = runtime_vars.net.getEdge(start_edgeID)
        if goal_edgeID != "":
            if '*' in goal_edgeID:
                # TODO: Dirty
                import random
                candidates = [x for x in runtime_vars.net.getEdges() if 'out' in x.getID() and x.getID() != 'out0']
                goal_edgeID = random.choice(candidates).getID()

            goal_edge = runtime_vars.net.getEdge(goal_edgeID)
            route_edges = [
                edge.getID()
                for edge in runtime_vars.net.getOptimalPath(start_edge, goal_edge)[0]
            ]
            runtime_vars.route_edges = [
                edge.getID()
                for edge in runtime_vars.net.getOptimalPath(
                    start_edge, goal_edge, withInternal=True
                )[0]
            ]
        else:
            route_edges = [start_edgeID]
            runtime_vars.route_edges = [start_edgeID]

        departPos = (
            runtime_vars.config.vehicle.start_offset
            if runtime_vars.config.vehicle.start_offset
            else "free"
        )
        departSpeed = (
            runtime_vars.config.vehicle.start_velocity
            if runtime_vars.config.vehicle.start_velocity
            else "random"
        )
        departLane = (
            runtime_vars.config.vehicle.lane_offset
            if runtime_vars.config.vehicle.lane_offset
            else "free"
        )

        runtime_vars.traci.route.add(routeID, route_edges)
        runtime_vars.traci.vehicle.add(
            egoID,
            routeID,
            departPos=departPos,
            departLane=departLane,
            departSpeed=departSpeed,
        )
        steps = 1
        while True:
            runtime_vars.sumo_engine.simulationStep()
            if traci.vehicle.getRoadID(runtime_vars.config.simulation.egoID) == "":
                steps += 1
                continue
            break

        logging.debug(
            f"Ego init: {egoID} on edge {start_edgeID} towards {route_edges[-1]} edge after {steps} step(s)."
        )

    def _insert_traffic(self):
        period = runtime_vars.config.variation.traffic_routing_period
        start_edges = runtime_vars.config.variation.traffic_routing_start_edges
        goal_strat = runtime_vars.config.variation.traffic_routing_goal_strategy

        edges = [
            edge
            for edge in runtime_vars.net.getEdges(withInternal=False)
            if edge.getLength() > 1.0
        ]
        edgeIDs = [edge.getID() for edge in edges]

        if runtime_vars.config.simulation.init_ego_strategy != "roundabout":
            edgeIDs_goal = [int(edgeID) for edgeID in edgeIDs if "#" not in edgeID]
        else:
            edgeIDs_goal = edgeIDs

        num_inserts = int(self._time_since_last_insert // period)
        if num_inserts > 0:
            self._time_since_last_insert = self._time_since_last_insert % period
            for i in range(num_inserts):
                if goal_strat == "intersection" and start_edges is None:
                    start_edge = runtime_vars.np_random_traffic.choice(
                        [str(edge) for edge in range(max(edgeIDs_goal))]
                    )
                elif goal_strat == "roundabout" and start_edges is None:
                    start_edge = runtime_vars.np_random_traffic.choice(
                        [id for id in edgeIDs_goal if "in" in id]
                    )
                else:
                    start_edge = runtime_vars.np_random_traffic.choice(start_edges)

                target_edge = self._calc_target_edge(
                    start_edge, edgeIDs_goal, goal_strat
                )
                route_edges = [start_edge]
                if (
                        target_edge
                ):  # If None, we only drive the current edge from start to finish
                    route_edges += [target_edge]

                routeID = f"route_trafficRunning_{runtime_vars.time_step}_{i}"
                vehID = f"trafficRunning_{runtime_vars.time_step}_{i}"
                runtime_vars.traci.route.add(routeID, route_edges)

                runtime_vars.traci.vehicle.add(
                    vehID,
                    routeID,
                    departPos="base",
                    departLane="free",
                    departSpeed="random",
                )

                logging.debug(
                    f"Traffic spawn: {vehID} on edge {start_edge} towards {route_edges[-1]} edge."
                )
        else:
            self._time_since_last_insert += runtime_vars.config.simulation.dt

    def _action_loop(self, action):
        for i in range(runtime_vars.config["simulation"]["steps_per_action"]):
            self._actions.step(action)
            if runtime_vars.sumo_engine.simulationStep(): return self._dummy_return()
            runtime_vars.traffic_manager.step()
            self.time_step += 1
            runtime_vars.time_step = self.time_step

            if runtime_vars.vehicle.speed == 0.0:
                self.standing_still_for += 1
            else:
                self.standing_still_for = 0

        if (
                runtime_vars.config["simulation"]["render"]
                and runtime_vars.config["simulation"]["render_navigation"]
        ):
            self._render_navigation()
            self._render_goals()

        # Insert new traffic
        if runtime_vars.config.variation.traffic_routing:
            self._insert_traffic()

    def _create_vehicle(self):
        vehicle_config = runtime_vars.config["vehicle"]
        egoID = runtime_vars.config["simulation"]["egoID"]
        dt = runtime_vars.config["simulation"]["dt"]
        if vehicle_config["dynamics_model"] in [
            DynamicsModel.KS,
            DynamicsModel.ST,
            DynamicsModel.STD,
            DynamicsModel.MB,
        ]:
            vehicle = TUMVehicle(
                vehicle_config, egoID, runtime_vars.traffic_manager, dt
            )
        elif vehicle_config["dynamics_model"] == DynamicsModel.TPS:
            vehicle = TPSVehicle(
                vehicle_config, egoID, runtime_vars.traffic_manager, dt
            )
        else:
            raise NotImplementedError

        waypoint_manager = WaypointManager()
        vehicle.attach_waypoint_manager(waypoint_manager)
        sub_goal_manager = SubGoalManager()
        vehicle.attach_sub_goal_manager(sub_goal_manager)
        traffic_randomizer = TrafficRandomizer()
        vehicle.attach_traffic_randomizer(traffic_randomizer)

        return vehicle

    @property
    def observation_space(self):
        if self._observer is not None:
            return self._observation_space

        observation_config = runtime_vars.config["observations"]
        observers = []
        for obs_name in observation_config["observers"]:
            if obs_name == Observer.EgoVehicle:
                observers.append(EgoVehicleObserver())
            elif obs_name == Observer.RadiusVehicle:
                observers.append(RadiusVehicleObserver())
            elif obs_name == Observer.Waypoints:
                observers.append(WaypointObserver())
            elif obs_name == Observer.SubGoals:
                observers.append(SubGoalObserver())
            elif obs_name == Observer.BirdsEye:
                observers.append(BirdsEyeObserver())
            elif obs_name == Observer.AvailableOptions:
                observers.append(AvailableOptionsObserver())
            elif obs_name == Observer.RoadShape:
                observers.append(RoadShapeObserver())
            else:
                raise NotImplementedError
        self._observer = MultiObserver(*observers)
        self._observation_space = self._observer.space

        # Adjust observation space to cwh if needed
        if (
                runtime_vars.config.observations.cwh
                and len(self._observation_space.shape) == 3
        ):
            import gym.spaces

            shape = self._observation_space.shape
            low = np.rollaxis(self._observation_space.low, 2)
            high = np.rollaxis(self._observation_space.high, 2)
            self._observation_space = gym.spaces.Box(low=low, high=high)

        # TODO: into observers
        return self._observation_space

    @property
    def action_space(self):
        if self._action_space is not None:
            return self._action_space

        actions_config = runtime_vars.config["actions"]

        if actions_config["space"] in [ActionSpace.Continuous, ActionSpace.Discretized]:
            self._actions = DirectActions()
        elif actions_config["space"] == ActionSpace.Semantic:
            self._actions = SemanticActions()
        else:
            raise NotImplementedError

        self._action_space = self._actions.space
        return self._action_space

    def _render_goals(self):
        lambda_poiID = lambda goal: f"goal_{goal.position}"

        sub_goals = runtime_vars.vehicle.sub_goals

        if len(self._rendered_goalIDs) == 0:
            for goal in sub_goals:
                try:
                    id = lambda_poiID(goal)
                    runtime_vars.traci.polygon.add(
                        id,
                        polygon_circle_shape(goal.position, 2.0, num_vert=3),
                        (255, 255, 0),
                        fill=True,
                        layer=9,
                        lineWidth=1,
                    )
                    self._rendered_goalIDs.append(id)
                except TraCIException:
                    pass
        else:
            i = 0
            if len(sub_goals) == 0:
                return
            id = lambda_poiID(sub_goals[0])
            while id != self._rendered_goalIDs[i]:
                runtime_vars.traci.polygon.remove(self._rendered_goalIDs[i])
                i += 1

            self._rendered_goalIDs = self._rendered_goalIDs[i:]

    def _render_navigation(self):
        lambda_poiID = lambda waypoint: f"wp_{waypoint.position}"

        waypoints = runtime_vars.vehicle.waypoints

        do_render = True
        if len(self._rendered_waypointIDs) > 0:
            first_waypointID = lambda_poiID(waypoints[0])
            last_waypointID = lambda_poiID(waypoints[-1])
            if (
                    first_waypointID == self._rendered_waypointIDs[0]
                    and last_waypointID == self._rendered_waypointIDs[-1]
            ):
                do_render = False
            else:
                for id in self._rendered_waypointIDs:
                    # if poiID not in waypointIDs:
                    try:
                        runtime_vars.traci.polygon.remove(id)
                    except TraCIException:
                        pass
                self._rendered_waypointIDs = []

        if len(waypoints) > 0 and do_render:
            # Render new navigation
            for waypoint in waypoints:
                # if poiID not in self.rendered_waypoints:
                try:
                    id = lambda_poiID(waypoint)

                    runtime_vars.traci.polygon.add(
                        id,
                        polygon_circle_shape(waypoint.position, 1.0),
                        (255, 0, 0),
                        fill=True,
                        layer=10,
                        lineWidth=1,
                    )
                    self._rendered_waypointIDs.append(id)
                except TraCIException:
                    pass

    @property
    def _reward_done_info(self):
        # Goal
        goal_edgeID = runtime_vars.route_edges[-1]
        goal_edge_length = runtime_vars.net.getEdge(goal_edgeID).getLength()
        distance_to_goal = traci.vehicle.getDrivingDistance(
            runtime_vars.config.simulation.egoID, goal_edgeID, goal_edge_length,
        )
        at_goal = distance_to_goal < 5.0

        # Collision
        ego_collided = collision_detection.check_collision()

        # Off-route
        if runtime_vars.config.actions.space == ActionSpace.Semantic and not at_goal:
            # Check if we're out of waypoints
            off_route = len(runtime_vars.vehicle.waypoints) <= 1
        else:
            from sumolib.geomhelper import distancePointToPolygon

            distance_to_lane = distancePointToPolygon(
                runtime_vars.vehicle.position,
                runtime_vars.net.getLane(runtime_vars.vehicle.laneID).getShape(),
            )
            off_route = runtime_vars.vehicle.laneIndex == -1073741824

            if distance_to_lane > 5.0:
                off_route = True

            route_idx = runtime_vars.traci.vehicle.getRouteIndex(runtime_vars.config.simulation.egoID)
            route_edgeID = runtime_vars.traci.vehicle.getRoute(runtime_vars.config.simulation.egoID)[route_idx]
            edgePoly = runtime_vars.net.getEdge(route_edgeID).getShape()
            route_dist = sumolib.geomhelper.distancePointToPolygon(runtime_vars.vehicle.position, edgePoly)

            if route_dist > 20.0:
                off_route = True

        # Check if driving in circles
        driving_in_circles = False
        if abs(runtime_vars.vehicle.angle) > 4 * 2 * np.pi:
            driving_in_circles = True

        # Timeouts
        timeout = (
                self.time_step > runtime_vars.config["simulation"]["max_time_steps"]
        )

        timeout_standing_still = (
                self.standing_still_for
                > runtime_vars.config["simulation"]["stand_still_timeout_after"]
        )

        ### Reward
        reward = 0.0
        reward_config = runtime_vars.config["reward"]
        if at_goal:
            reward = reward_config["goal_reward"]
        elif ego_collided:
            reward = reward_config["collision_penalty"] if ego_collided else 0.0
        elif off_route:
            reward = reward_config["off_route_penalty"]
        elif timeout or timeout_standing_still:
            reward = reward_config["timeout_penalty"]
        elif driving_in_circles:
            reward = reward_config["driving_circles_penalty"]
        else:
            # Speed reward
            reward += (
                    reward_config["speed_reward"]
                    * runtime_vars.vehicle.speed
                    / runtime_vars.vehicle.v_max
            )
            if runtime_vars.vehicle.speed == 0.0:
                reward += reward_config["stand_still_penalty"]

            # Subgoal reward
            if len(self._last_sub_goals) > len(runtime_vars.vehicle.sub_goals):
                reward += reward_config["sub_goal_reward"]

        ### Done
        done = timeout or timeout_standing_still or at_goal or off_route or ego_collided or driving_in_circles

        ### Info
        info = dict(
            cost=(1 if ego_collided else 0),  # Many safety-related methods need this
            level_seed=runtime_vars.level_seed,  # Methods like PLR make use of this
            time_step=runtime_vars.time_step,
            reached_goal=at_goal,
            collision=ego_collided,  # For stats
            timeout=timeout,
            timeout_standing_still=timeout_standing_still,
            off_route=off_route,
        )

        return reward, done, info

    def _record(self):
        assert runtime_vars.config.simulation.render, "Can only record when simulation.render == True"

        parent_path = runtime_vars.config.simulation.render_record
        os.makedirs(parent_path, exist_ok=True)
        cur_path = pjoin(parent_path, str(runtime_vars.episode_count))
        if not os.path.isdir(cur_path):
            # Create folder
            os.makedirs(cur_path)

            last_path = pjoin(parent_path, str(runtime_vars.episode_count - 1))
            if runtime_vars.config.simulation.render_record_video and os.path.isdir(last_path):
                import ffmpeg
                (
                    ffmpeg
                    .input(f"{last_path}/*.png", pattern_type='glob', framerate=round(1.0 / runtime_vars.config.simulation.dt))
                    .output(f'{last_path}/video.mp4')
                    .run()
                )

        runtime_vars.traci.gui.screenshot(traci.gui.DEFAULT_VIEW, f"{pjoin(cur_path, str(runtime_vars.time_step))}.png")
