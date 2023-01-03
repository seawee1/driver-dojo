import sys

from driver_dojo.core.carla_engine import CarlaEngine
from driver_dojo.observer.carla import CarlaCameraObserver


import gym
import traci
import logging
import logging.config

from pyglet.window import key
from omegaconf import DictConfig, OmegaConf
import string
import numpy as np
from omegaconf import OmegaConf
from visdom import Visdom

from driver_dojo.actions import DirectActions, SemanticActions, BaseActions
from driver_dojo.common import TrafficManager
from driver_dojo.common.road_manager import StreetMap
from driver_dojo.core import SUMOEngine
from driver_dojo.graphics.renderer import Renderer
from driver_dojo.scenarios.basic_scenarios import BasicScenario
from driver_dojo.scenarios.scenario_manager import ScenarioManager
from driver_dojo.vehicle.attachments import SubGoalAttachment, WaypointAttachment  # , TrafficRandomizerAttachment
from driver_dojo.vehicle import TUMVehicle, GodVehicle, BaseVehicle
import driver_dojo.common.collision_detection as collision_detection
from driver_dojo.core.config import *
from driver_dojo.core.types import ActionSpace
from driver_dojo.observer import (
    MultiObserver,
    EgoVehicleObserver,
    TrafficObserver,
    WaypointObserver,
    AvailableOptionsObserver,
    SubGoalObserver,
    BirdsEyeObserver,
    RoadShapeLidarObserver,
    RoadObserver,
    BaseObserver,
)


class DriverDojoEnv(gym.Env):
    metadata = {"render_modes": ['human', 'rgb_array', 'rgb_array_map']}

    def __init__(
            self,
            config: DictConfig = None,
            _config: DictConfig = None,
            render_mode='rgb_array'
    ):
        c = Config()
        if _config:
            c = OmegaConf.merge(c, _config)
        if config:
            c = OmegaConf.merge(c, config)
        self.config = c

        os.makedirs(self.config.simulation.work_path, exist_ok=True)  # Create work directory

        logging_level = logging.INFO if self.config.debug.verbose else logging.CRITICAL
        logging.basicConfig(level=logging_level)

        literals = string.ascii_uppercase + string.digits  # Set identifying label for SUMOEngine
        self.sumo_label: str = "".join(
            [literals[np.random.randint(0, len(literals))] for _ in range(20)]
        )

        self.sumo_engine: SUMOEngine = SUMOEngine(self.config, self.sumo_label)
        self.street_map: StreetMap = StreetMap()
        self.traffic_manager: TrafficManager = TrafficManager(self.config)
        self.scenario_manager: ScenarioManager = ScenarioManager(self.config)

        self.renderer = None
        if self.config.rendering.human_mode or (Observer.BirdsEye in self.config.observations.observers) or self.config.simulation.interactive:  # If we need a Renderer
            self.renderer: Renderer = Renderer(self.config, self.traffic_manager, self.street_map)

        self.vehicle: BaseVehicle = self._setup_vehicle()
        observer_setup: Tuple[MultiObserver, List[BaseObserver]] = self._setup_observer()
        self.observer, self.carla_observers = observer_setup
        self.actions: BaseActions = self._setup_actions()
        self.carla_engine = None
        if self.config.simulation.carla_co_simulation:
            self.carla_engine: CarlaEngine = CarlaEngine(self.config, self.traffic_manager, self.carla_observers)

        self.observation_space: np.ndarray = self.observer.observation_space  # TODO: rename to `.space` for clarity
        self.action_space: np.ndarray = self.actions.action_space

        self.scenario: Optional[BasicScenario] = None  # Helper variables
        self._time_step: int = 0
        self._standing_still_since: int = 0
        self._last_sub_goals: List = list()
        self._vis = None
        self._episode_count = -1
        self.render_mode = render_mode
        self.traci = None

        if self.config.debug.debug:  # Setup visdom for debugging
            vis_args = dict(
                server=self.config.debug.visdom_host,
                port=self.config.debug.visdom_port,
                log_to_filename=self.config.debug.visdom_log_path,
                raise_exceptions=True
            )
            try:
                self._vis = Visdom(**vis_args)
            except ConnectionError:  # If no visdom server is running, we log it to a offline file. Replay with `self._viz.replay_log(file_path)`
                raise ConnectionError('Please start a visdom server first to use the debugging mode via issuing `visdom` in a command line!')
                # TODO: Right now, visdom seems to be bugged and not work properly in offline mode
                # if vis_args['log_to_filename'] is None:
                #     vis_args['log_to_filename'] = f'visdom_{self.sumo_label}'
                # vis_args['offline'] = True
                # self._vis = Visdom(**vis_args)

        logging.info(OmegaConf.to_yaml(self.config))

    def reset(self, seed=None, **kwargs):
        super().reset(seed=seed)

        self.scenario: BasicScenario = self.scenario_manager.step()  # Get new scenario
        self.traci = self.sumo_engine.reset(self.scenario)  # Load it into the SUMO engine
        if self.config.simulation.carla_co_simulation:
            self.carla_engine.reset(self.scenario)

        self.traffic_manager.reset(self.traci)  # Reset the traffic state
        self.scenario.initialize(self.traci)  # Initialize the scenario (some traci.vehicle.add calls in most cases)
        while self.config.simulation.egoID not in self.traffic_manager.actor_ids:  # Run simulation until ego was added
            self.sumo_engine.simulationStep()
            if self.carla_engine is not None:
                self.carla_engine.simulationStep()
            self.traffic_manager.step()

        self.street_map.reset(self.scenario)  # Reload lane-graph and stuff
        self.vehicle.reset()
        if self.renderer is not None:
            self.renderer.reset(self.traci, self.vehicle, self.scenario)  # Has to be done after RoadManager.reset() due to lane_graph being used for rendering
            self.renderer.step()
        self.actions.reset()
        self.observer.reset()

        self._time_step = 0
        self._episode_count += 1
        self._standing_still_since = 0  # Used for 'standing still' timeout
        self._last_sub_goals = self.vehicle.sub_goals  # Used for reward calculation when consuming sub-goals
        # self.traci.vehicle.subscribeContext('ego', traci.constants.CMD_GET_VEHICLE_VARIABLE, dist=50.0)
        # self.traci.vehicle.addSubscriptionFilterTurn(downstreamDist=100.0, foeDistToJunction=20.0)
        obs = self.observer.step()
        return obs

    def _overwrite_action_with_input(self, action):
        keyboard_state = self.renderer.keyboard_state
        if keyboard_state is None or len(keyboard_state) == 0:
            return action

        key_map = self.config.simulation.keyboard_to_action  # Defines which actions to trigger by which keyboard input

        if key.R in keyboard_state and keyboard_state[key.R]:  # Reset the environment on `R`
            self.reset()
            return self.actions.noop_action
        elif key.P in keyboard_state and keyboard_state[key.P]:
            while True:
                import time
                time.sleep(10)

        relevant_keys_pressed = [k for k, v in keyboard_state.items() if v and k in key_map.keys()]
        if len(relevant_keys_pressed) == 0:  # Do a NOOP action
            action = self.actions.noop_action
        else:  # Do one of the actions
            action = key_map[relevant_keys_pressed[0]]
        return action

    def step(self, action):
        if self.config.simulation.interactive:  # We overwrite with keyboard inputs
            action = self._overwrite_action_with_input(action)

        self._action_loop(action)  # Perform the action
        if self.renderer is not None:
            self.renderer.step()

        if self.config.debug.debug:
            self._debug_routine()

        obs = self.observer.step()
        reward, done, info = self._reward_done_info(self.scenario)

        self._last_sub_goals = self.vehicle.sub_goals
        return obs, reward, done, info

    def render(self, mode="rgb_array"):
        if self.renderer is None:
            self.renderer: Renderer = Renderer(self.config, self.traffic_manager, self.street_map)
            self.renderer.reset(self.traci, self.vehicle, self.scenario)
            self.renderer.step()

        if mode == 'human':
            self.renderer.visible = True
        elif mode == 'rgb_array':
            return self.renderer.get_image()
        elif mode == 'rgb_array_map':
            return self.renderer.get_image(global_view=True)

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
        self.sumo_engine.close()

    def _action_loop(self, action):
        for i in range(self.config.simulation.steps_per_action):
            self.actions.step(action)  # Do the action
            self.sumo_engine.simulationStep()
            if self.carla_engine is not None:
                self.carla_engine.simulationStep()
            self.scenario.step(self.traci)  # Perform whatever events the scenario defines
            self.traffic_manager.step()  # Has to be stepped inside here, due to it being closely connected to SUMOEngine.simulationStep() for correct functioning
            self._time_step += 1

            if self.vehicle.velocity == 0.0:
                self._standing_still_since += 1
            else:
                self._standing_still_since = 0

    def _setup_vehicle(self):
        vehicle_config = self.config.vehicle
        if vehicle_config.dynamics_model in [
            DynamicsModel.KS,
            DynamicsModel.ST,
            DynamicsModel.STD,
            DynamicsModel.MB,
        ]:
            vehicle = TUMVehicle(self.config, self.traffic_manager)
        elif vehicle_config.dynamics_model == DynamicsModel.God:
            vehicle = GodVehicle(self.config, self.traffic_manager)
        else:
            raise NotImplementedError

        if self.config.actions.space == ActionSpace.Semantic:  # We only need waypoints for semantic actions
            waypoint_manager = WaypointAttachment(self.config, vehicle, self.street_map)
            vehicle.attach(waypoint_manager)
        sub_goal_manager = SubGoalAttachment(self.config, vehicle, self.street_map)
        vehicle.attach(sub_goal_manager)
        # traffic_randomizer = TrafficRandomizerAttachment()  # TODO: fix it
        # vehicle.attach(traffic_randomizer)
        return vehicle

    def _setup_observer(self) -> Tuple[MultiObserver, List[BaseObserver]]:
        observation_config = self.config.observations
        observers = []
        observer_args = [self.config, self.vehicle, self.traffic_manager]
        for obs_name in observation_config["observers"]:
            if obs_name == Observer.EgoVehicle:
                o = EgoVehicleObserver(*observer_args)
            elif obs_name == Observer.RadiusVehicle:
                o = TrafficObserver(*observer_args)
            elif obs_name == Observer.Waypoints:
                o = WaypointObserver(*observer_args)
            elif obs_name == Observer.SubGoals:
                o = SubGoalObserver(*observer_args)
            elif obs_name == Observer.BirdsEye:
                o = BirdsEyeObserver(*observer_args, self.renderer)
            elif obs_name == Observer.AvailableOptions:
                o = AvailableOptionsObserver(*observer_args)
            elif obs_name == Observer.RoadShape:
                o = RoadShapeLidarObserver(*observer_args, self.street_map)
            elif obs_name == Observer.Road:
                o = RoadObserver(*observer_args, self.street_map)
            else:
                raise NotImplementedError
            observers.append(o)

        carla_observers = []
        for carla_obs_config in observation_config.carla_observers:
            if carla_obs_config.sensor_type in [CarlaSensor.RGBCamera, CarlaSensor.DepthCamera]:
                o = CarlaCameraObserver(*observer_args, carla_obs_config)
            else:
                raise NotImplementedError
            observers.append(o)
            carla_observers.append(o)

        return MultiObserver(*observers, config=self.config, vehicle=self.vehicle, traffic_manager=self.traffic_manager), carla_observers

    def _setup_actions(self) -> BaseActions:
        actions_config = self.config.actions

        if actions_config.space in [ActionSpace.Continuous, ActionSpace.Discretized]:
            actions = DirectActions(self.config, self.vehicle)
        elif actions_config.space == ActionSpace.Semantic:
            actions = SemanticActions(self.config, self.vehicle)
        else:
            raise NotImplementedError
        return actions

    def _reward_done_info(self, scenario):
        goal_edgeID = scenario.route[-1]  # Goal
        goal_edge_length = scenario.sumo_net.getEdge(goal_edgeID).getLength()
        distance_to_goal = traci.vehicle.getDrivingDistance(
            self.config.simulation.egoID, goal_edgeID, goal_edge_length,
        )
        at_goal = distance_to_goal < 10.0

        ego_collided = collision_detection.check_collision(self.config.simulation.egoID, self.traffic_manager)  # Collision

        off_route = True
        lanes = self.scenario.sumo_net.getNeighboringLanes(*self.vehicle.location[:2], 2.0)
        if len(lanes) > 0:
            for lane, dist in lanes:
                if lane.getEdge().isSpecial():
                    off_route = False
                    break
                elif lane.getEdge().getID() in self.scenario.route:
                    off_route = False
                    break

        # Check if driving in circles
        driving_in_circles = False
        # if abs(self.world.vehicle.rotation[1]) > 4 * 2 * np.pi:
        #     # TODO: I don't think this works in its current state
        #     # driving_in_circles = True
        #     pass

        timeout = (
                self._time_step > self.config.simulation.max_time_steps
        )

        timeout_standing_still = (
                self._standing_still_since
                > self.config.simulation.stand_still_timeout_after
        )

        # Reward
        reward = 0.0
        reward_config = self.config.reward
        if at_goal:
            reward = reward_config.goal_reward
        elif ego_collided:
            reward = reward_config.collision_penalty
        elif off_route:
            reward = reward_config.off_route_penalty
        elif timeout or timeout_standing_still:
            reward = reward_config.timeout_penalty
        elif driving_in_circles:
            reward = reward_config.driving_circles_penalty
        else:
            # Speed reward
            reward += (
                    reward_config.speed_reward
                    * self.vehicle.velocity
                    / self.vehicle.velocity_max
            )
            if self.vehicle.velocity == 0.0:
                reward += reward_config.stand_still_penalty

            # Sub-goal reward
            if len(self._last_sub_goals) > len(self.vehicle.sub_goals):
                reward += reward_config.sub_goal_reward

        # Done
        done = timeout or timeout_standing_still or at_goal or off_route or ego_collided or driving_in_circles

        # Info
        info = dict(
            cost=int(timeout or timeout_standing_still or off_route or ego_collided or driving_in_circles),
            time_step=self._time_step,
            road_seed=scenario.net_seed,
            traffic_seed=scenario.traffic_seed,
            reached_goal=at_goal,
            collision=ego_collided,
            timeout=timeout,
            timeout_standing_still=timeout_standing_still,
            off_route=off_route,
        )

        return reward, done, info

    def _debug_routine(self):
        # Note: install via `pip install -e git://github.com/facebookresearch/visdom.git#egg=visdom` instead of `pip install visdom`
        first_step = self._time_step == self.config.simulation.steps_per_action
        if first_step:  # Log image of street network
            #map_im = np.rollaxis()
            map_im = self.renderer.get_image(global_view=True)
            opts = dict(
                caption=f'road seed {self.scenario.net_seed}',
                store_history=True,
                jpgquality=80
            )
            self._vis.image(
                map_im,
                win='road layout',
                opts=opts,
            )

        if not ((self._time_step - 1) % self.config.debug.step_period) == 0:  # Only log every x steps
            return

        #cur_im = np.rollaxis( 2)  # Render current rendered scene
        cur_im = self.renderer.get_image()
        self._vis.image(
            cur_im,
            win=f'Render, episode {self._episode_count}' if self.config.debug.visdom_split_episodes else 'render',
            opts=dict(
                caption=f'traffic seed {self.scenario.traffic_seed}',
                store_history=True,
                jpgquality=100
            )
        )

        feature_scaling = self.config.observations.feature_scaling is not None
        relative_features = self.config.observations.relative_to_ego
        observer_config = self.config.observations.observers

        if feature_scaling:  # For now, we don't support debugging with normalized features
            logging.info("Didn't log observer outputs to visdom because Config.observations.feature_scaling != None.")
            return

        if Observer.RadiusVehicle in observer_config:
            observer = self.observer.get_observer(TrafficObserver)  # Get observer outputs, format values
            obs = observer.step()
            obs_labels = observer.explain()
            xs = [obs[i] for i, x in enumerate(obs_labels) if x == 'x']
            ys = [obs[i] for i, x in enumerate(obs_labels) if x == 'y']
            print(xs)
            print(ys)
            rotations = [obs[i] for i, x in enumerate(obs_labels) if x == 'rotation']

            actor_pos = np.array([0, 0]) if relative_features else self.vehicle.location[:2]
            X = np.array([[x, y] for x, y in zip(xs, ys)])
            mask = np.sum(np.abs(X), axis=1) != 0  # Filter out dummy values
            X = X[mask]
            X = np.vstack((actor_pos, X))
            rotations = [x for i, x in enumerate(rotations) if mask.flatten()[i]]
            rotations = [round(x, 2) for x in rotations]
            rotations = [0.0] + rotations

            obs_radius = self.config.observations.rvo_radius
            axis_range = np.array([actor_pos - obs_radius, actor_pos + obs_radius])
            self._vis.scatter(
                X=X,
                opts=dict(
                    title='RadiusVehicleObserver',
                    xtickmin=axis_range[0][0],
                    xtickmax=axis_range[1][0],
                    ytickmin=axis_range[0][1],
                    ytickmax=axis_range[1][1],
                    textlabels=[f'{x}' for x in rotations],
                ),
                win='RadiusVehicleObserver',
            )

        if Observer.EgoVehicle in observer_config:
            observer = self.observer.get_observer(EgoVehicleObserver)
            obs = observer.step()
            obs_labels = observer.explain()
            text = [f'{label}: {val}' for label, val in zip(obs_labels, obs)]
            print('EgoVehicle observations')
            print(text)  # TODO: Log to visdom

        for observer_type in [Observer.Waypoints, Observer.SubGoals]:
            if observer_type not in observer_config:
                continue

            observer = self.observer.get_observer(WaypointObserver if observer_type == Observer.Waypoints else SubGoalObserver)
            obs = observer.step()
            obs_labels = observer.explain()
            xs = [obs[i] for i, x in enumerate(obs_labels) if x == 'x']
            ys = [obs[i] for i, x in enumerate(obs_labels) if x == 'y']
            match = [obs[i] for i, x in enumerate(obs_labels) if x == 'match_angle']
            dist = [obs[i] for i, x in enumerate(obs_labels) if x == 'dist']
            X = np.array([[x, y] for x, y in zip(xs, ys)])
            textlabels = [f'{round(x, 2), round(y, 2)}' for x, y in zip(dist, match)]

            # Define axis range
            max_range = (self.config.observations.wp_num * self.config.observations.wp_sampling_dist) if observer_type == Observer.Waypoints else self.config.observations.sub_goal_dist_max
            x_min = -max_range if relative_features else self.vehicle.location[0] - max_range
            x_max = max_range if relative_features else self.vehicle.location[0] + max_range
            y_min = 0.0 if relative_features else self.vehicle.location[1]
            y_max = max_range if relative_features else self.vehicle.location[1] + max_range

            self._vis.scatter(
                X=X,
                opts=dict(
                    title='WaypointObserver' if observer_type == Observer.Waypoints else 'SubGoalObserver',
                    xtickmin=x_min,
                    xtickmax=x_max,
                    ytickmin=y_min,
                    ytickmax=y_max,
                    textlabels=textlabels,
                ),
                win='WaypointObserver' if observer_type == Observer.Waypoints else 'SubGoalObserver'
            )

        if Observer.AvailableOptions in observer_config:
            observer = [x for x in self.observer.obs_members['vector'] if isinstance(x, AvailableOptionsObserver)][0]
            obs = observer.step()
            obs_labels = observer.explain()
            text = [f'{label}: {val}' for label, val in zip(obs_labels, obs)]
            print('AvailableOptions observations')
            print(text)  # TODO: Log to visdom

        if Observer.RoadShape in observer_config:
            observer = self.observer.get_observer(RoadShapeLidarObserver)
            obs = observer.step()

            max_range = self.config.observations.rs_ray_dist
            x_min = -max_range if relative_features else self.vehicle.location[0] - max_range
            x_max = max_range if relative_features else self.vehicle.location[0] + max_range
            y_min = 0.0 if relative_features else self.vehicle.location[1]
            y_max = max_range if relative_features else self.vehicle.location[1] + max_range
            X = obs.reshape((-1, 2))
            self._vis_win_road = self._vis.scatter(
                X=X,
                opts=dict(
                    title="RoadShapeObserver",
                    xtickmin=x_min,
                    xtickmax=x_max,
                    ytickmin=y_min,
                    ytickmax=y_max,
                ),
                win="RoadShapeObserver",
            )

        if Observer.BirdsEye in observer_config:
            observer = self.observer.get_observer(BirdsEyeObserver)
            obs = observer.step()
            obs = np.rollaxis(obs, 2)
            self._vis.image(
                obs,
                opts=dict(
                    caption='BirdsEyeObserver',
                    jpgquality=100
                ),
                win='BirdsEyeObserver'
            )

        opts = dict(
            caption=f'traffic seed {self.scenario.traffic_seed}',
            store_history=True,
            jpgquality=100
        )
        for i, observer in enumerate(self.carla_observers):
            obs = observer.step()
            obs = np.rollaxis(obs, 2)
            self._vis.image(
                obs,
                opts=dict(
                    caption=f'CarlaObserver{i}',
                    store_history=True,
                    jpgquality=100
                ),
                win=f'CarlaObserver{i}'
            )

    # def _calc_target_edge(self, startID, edgeIDs, strat):
    #     # TODO: I don't like this whole . Can definitely be implemented more nicely.
    #     # Find out proper goal edge
    #     if strat == "min-or-0":  # Strategy for Highway Scenarios
    #         if "-" in startID:
    #             target_edge = str(min(edgeIDs))
    #         else:
    #             target_edge = str(0)
    #     elif strat == "highway-exit":
    #         if "-" in startID:
    #             if startID == "-3":
    #                 target_edge = "-3"
    #             else:
    #                 target_edge = self.world.traffic_seed_sampler.choice(["-3", "-2"])
    #             # If this is the case, we have a splitting road that is converted into "0$0", "0$1" and "0#2"
    #         else:
    #             target_edge = "0#0"
    #     elif strat == "same":
    #         target_edge = None
    #     elif strat == "intersection":
    #         if "-" in startID:
    #             return None
    #         num_edges = max(edgeIDs) + 1
    #         target_edges = [
    #             -x for x in range(0, num_edges) if abs(int(startID)) != abs(x)
    #         ]
    #         target_edge = self.world.traffic_seed_sampler.choice(target_edges)
    #         if target_edge == 0:
    #             target_edge = str("-0")
    #         else:
    #             target_edge = str(target_edge)
    #     elif strat == "roundabout":
    #         if "out" in startID:
    #             return None
    #         edgeIDs = [id for id in edgeIDs if "out" in id]
    #         target_edge = self.world.traffic_seed_sampler.choice(edgeIDs)
    #     else:
    #         raise NotImplementedError
    #
    #     if target_edge == startID:
    #         return None
    #     return target_edge
    #
    # def _init_traffic(self):
    #     # Read in from config
    #     dest_strat = self.world.config.simulation.init_traffic_goal_strategy
    #     exclude_edgeIDs = (
    #         self.world.config.simulation.init_traffic_exclude_edges
    #         if self.world.config.simulation.init_traffic_exclude_edges
    #         else []
    #     )
    #     traffic_spread = self.world.config.simulation.init_traffic_spread * 1.0 / self._traffic_density_episode
    #
    #     edges = [
    #         edge
    #         for edge in self.world.net.getEdges(withInternal=False)
    #         if edge.getLength() > 1.0
    #     ]
    #     edgeIDs = [edge.getID() for edge in edges]
    #     edges_num_vehicles = dict()
    #     street_area = 0.0
    #     # Find out total area of street network
    #     for edge in edges:
    #         area = edge.getLength() * len(edge.getLanes())
    #         street_area += area
    #         edges_num_vehicles[edge.getID()] = area
    #
    #     # Define how many vehicles to spawn in total
    #     num_vehicles = int(round(street_area / traffic_spread))
    #     for edge in edges:
    #         if edge.getID() in exclude_edgeIDs:
    #             continue
    #         edges_num_vehicles[edge.getID()] = int(
    #             round(edges_num_vehicles[edge.getID()] / street_area * num_vehicles)
    #         )
    #
    #     # We have to remove those. These are edges part of a merge/split.
    #     # TODO: refactor
    #     if self.world.config.simulation.init_ego_strategy != "roundabout":
    #         edgeIDs_goal = [int(edgeID) for edgeID in edgeIDs if "#" not in edgeID]
    #     else:
    #         edgeIDs_goal = edgeIDs
    #
    #     for edgeID, num in edges_num_vehicles.items():
    #         if edgeID in exclude_edgeIDs:
    #             continue
    #
    #         target_edge = self._calc_target_edge(
    #             edgeID,
    #             edgeIDs_goal,
    #             self.world.config.simulation.init_traffic_goal_strategy,
    #         )
    #
    #         for i in range(num):
    #             routeID = f"route_trafficInit_{edgeID}_{i}"
    #             vehID = f"trafficInit_{edgeID}_{i}"
    #             route_edges = [edgeID]
    #             if (
    #                     target_edge
    #             ):  # If None, we only drive the current edge from start to finish
    #                 route_edges += [target_edge]
    #             self.world.traci.route.add(routeID, route_edges)
    #
    #             typeID = "DEFAULT_VEHTYPE" if not self.world.config.variation.traffic_vTypes else f"vehDist{self.world.traffic_seed_sampler.randint(0, self.world.config.variation.vTypeDistribution_num)}"
    #             self.world.traci.vehicle.add(
    #                 vehID,
    #                 routeID,
    #                 typeID=typeID,
    #                 departPos="random_free",
    #                 departLane="best",
    #                 departSpeed="random",
    #             )
    #
    #             logging.debug(
    #                 f"Traffic init: {vehID} on edge {edgeID} towards {route_edges[-1]}."
    #             )
    #
    # def _init_ego(self):
    #     egoID = self.world.config.simulation.egoID
    #     routeID = self.world.config.simulation.routeID
    #     start_edgeID = self.world.config.simulation.init_ego_start_edge
    #     goal_edgeID = self.world.config.simulation.init_ego_goal_edge
    #
    #     start_edge = self.world.net.getEdge(start_edgeID)
    #     if goal_edgeID != "":
    #         if '*' in goal_edgeID:
    #             # TODO: Dirty
    #             import random
    #             candidates = [x for x in self.world.net.getEdges() if 'out' in x.getID() and x.getID() != 'out0']
    #             goal_edgeID = random.choice(candidates).getID()
    #
    #         goal_edge = self.world.net.getEdge(goal_edgeID)
    #         route_edges = [
    #             edge.getID()
    #             for edge in self.world.net.getOptimalPath(start_edge, goal_edge)[0]
    #         ]
    #         self.world.route_edges = [
    #             edge.getID()
    #             for edge in self.world.net.getOptimalPath(
    #                 start_edge, goal_edge, withInternal=True
    #             )[0]
    #         ]
    #     else:
    #         route_edges = [start_edgeID]
    #         self.world.route_edges = [start_edgeID]
    #
    #     departPos = (
    #         self.world.config.vehicle.start_offset
    #         if self.world.config.vehicle.start_offset
    #         else "free"
    #     )
    #     departSpeed = (
    #         self.world.config.vehicle.start_velocity
    #         if self.world.config.vehicle.start_velocity
    #         else "random"
    #     )
    #     departLane = (
    #         self.world.config.vehicle.start_lane_offset
    #         if self.world.config.vehicle.start_lane_offset
    #         else "free"
    #     )
    #
    #     self.world.traci.route.add(routeID, route_edges)
    #     self.world.traci.vehicle.add(
    #         egoID,
    #         routeID,
    #         departPos=departPos,
    #         departLane=departLane,
    #         departSpeed=departSpeed,
    #     )
    #     steps = 1
    #     while True:
    #         self.world.sumo_engine.simulationStep()
    #         if traci.vehicle.getRoadID(self.world.config.simulation.egoID) == "":
    #             steps += 1
    #             continue
    #         break
    #
    #     logging.debug(
    #         f"Ego init: {egoID} on edge {start_edgeID} towards {route_edges[-1]} edge after {steps} step(s)."
    #     )
    #
    # def _insert_traffic(self):
    #     period = self.world.config.variation.traffic_routing_period / self._traffic_density_episode
    #     start_edges = self.world.config.variation.traffic_routing_start_edges
    #     goal_strat = self.world.config.variation.traffic_routing_goal_strategy
    #
    #     edges = [
    #         edge
    #         for edge in self.world.net.getEdges(withInternal=False)
    #         if edge.getLength() > 1.0
    #     ]
    #     edgeIDs = [edge.getID() for edge in edges]
    #
    #     if self.world.config.simulation.init_ego_strategy != "roundabout":
    #         edgeIDs_goal = [int(edgeID) for edgeID in edgeIDs if "#" not in edgeID]
    #     else:
    #         edgeIDs_goal = edgeIDs
    #
    #     num_inserts = int(self._time_since_last_insert // period)
    #     if num_inserts > 0:
    #         self._time_since_last_insert = self._time_since_last_insert % period
    #         for i in range(num_inserts):
    #             if goal_strat == "intersection" and start_edges is None:
    #                 start_edge = self.world.traffic_seed_sampler.choice(
    #                     [str(edge) for edge in range(max(edgeIDs_goal))]
    #                 )
    #             elif goal_strat == "roundabout" and start_edges is None:
    #                 start_edge = self.world.traffic_seed_sampler.choice(
    #                     [id for id in edgeIDs_goal if "in" in id]
    #                 )
    #             else:
    #                 start_edge = self.world.traffic_seed_sampler.choice(start_edges)
    #
    #             target_edge = self._calc_target_edge(
    #                 start_edge, edgeIDs_goal, goal_strat
    #             )
    #             route_edges = [start_edge]
    #             if (
    #                     target_edge
    #             ):  # If None, we only drive the current edge from start to finish
    #                 route_edges += [target_edge]
    #
    #             routeID = f"route_trafficRunning_{self.world.time_step}_{i}"
    #             vehID = f"trafficRunning_{self.world.time_step}_{i}"
    #             self.world.traci.route.add(routeID, route_edges)
    #
    #             self.world.traci.vehicle.add(
    #                 vehID,
    #                 routeID,
    #                 departPos="base",
    #                 departLane="free",
    #                 departSpeed="random",
    #             )
    #
    #             logging.debug(
    #                 f"Traffic spawn: {vehID} on edge {start_edge} towards {route_edges[-1]} edge."
    #             )
    #     else:
    #         self._time_since_last_insert += self.world.config.simulation.dt
