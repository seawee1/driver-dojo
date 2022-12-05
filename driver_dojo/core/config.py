import os
import sys
from omegaconf import MISSING
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Union
from os. path import realpath, abspath, join, dirname

from driver_dojo.core.types import (
    CarFollowingModel,
    DynamicsModel,
    CarModel,
    GeneratableRoad,
    FeatureScaling,
    Observer,
    ActionSpace,
    CarlaSensor,
)


@dataclass
class SimulationConfig:
    work_path: str = abspath(join(dirname(__file__), '..', '..', '.driver_dojo'))
    dt: float = 0.2  # Duration of one environment step
    steps_per_action: int = 1  # How many time steps to repeat one action
    max_time_steps: int = 300  # Maximum amount of environment steps
    stand_still_timeout_after: int = max_time_steps
    gui_xml_path: str = abspath(join(dirname(__file__), "..", "data", "gui.xml"))
    egoID: str = "ego"  # ID of ego vehicle
    routeID: str = "route_ego"  # ID of ego route
    demand_scale: float = 1.0  # Scale traffic amount. Alert! Don't set bigger than 1.0, as this will create multiple ego spawns!
    interactive: bool = False  # Used for manual_control.py
    keyboard_to_action: Optional[Dict] = None
    seed: Optional[int] = 0
    tls_off: bool = False  # If traffic lights are present, turn them all off
    traffic_manager_radius: float = 100.0  # How many meters' radius to keep track of traffic state around ego vehicle
    # filter_traffic: bool = False  # TODO: Filter observable traffic down to junction foes and ego followers and leaders
    time_to_impatience: float = 10.0  # How many seconds for non-ego driver to become impatient
    car_following_model: CarFollowingModel = CarFollowingModel.EIDM
    lateral_resolution: float = 0.72
    junction_ignore_ego: bool = False
    sumo_cmd_extra_args: str = ""  # TODO: Add to code.
    subgoals_only_after: bool = False  # If subgoals should only be added after an edge.
    # init_time: float = 0.0  # How many seconds to freeze/delay spawn of the ego vehicle and initialize traffic (delay in the case of init_ego=True)
    # init_traffic: bool = (
    #     False  # Initialize the network with traffic via traci.vehicle.add()
    # )
    # init_traffic_spread: float = (
    #     20.0  # Mean spread between spawned traffic vehicles per lane.
    # )
    # init_traffic_goal_strategy: Optional[
    #     str
    # ] = False  # How to select goal edge for spawned traffic
    # init_traffic_exclude_edges: Optional[List[str]] = None
    # init_ego: bool = False  # If ego spawning should be handled through traci.vehicle.add(). Alert: This might cause problems ego routes defined through the rou.xml file.
    # init_ego_start_edge: Optional[str] = None
    # init_ego_goal_edge: Optional[str] = None
    # init_ego_strategy: Optional[str] = None
    # info: bool = False  # Info logging activate
    # store_for_plotting: Optional[
    #     str
    # ] = None  # Will copy every map (.net.xml and .xodr file) to the folder specified by 'store_for_plotting'
    time_to_teleport: Optional[float] = -1
    carla_co_simulation: bool = False
    carla_path: str = (
        "C:\\Users\\seegras\\Portables\\CARLA_0.9.13\\WindowsNoEditor\\CarlaUE4.exe"
    )

@dataclass
class RenderingConfig:
    sumo_gui: bool = False
    carla_gui: bool = False  # TODO
    human_mode: bool = False
    width: int = 512
    height: int = 512
    radius: int = 50

# class EgoInitializationConfig:
#     enabled: bool = False
#     freeze_for: float = 0.0
#     from_edges: Optional[Tuple[str]] = None
#     to_edges: Optional[Tuple[str]] = None
#     position: Union[str, float] = '5.0'
#     lane: Union[str, float] = 'free'
#     speed: Union[str, float] = 'random'
#
# class TrafficInitializationConfig:
#     enabled: bool = False
#     edges: Optional[Tuple[str]] = None
#     edges_exclude: Optional[Tuple[str]] = None
#     exclude_ego_edge: bool = True
#     spread: float = 10.0
#     params: Tuple[str, str, str] = ('random_free', 'best', 'random')  # departPos, departLane, departSpeed
#
# class TrafficSpawnConfig:
#     enabled: bool = False
#     edges: Optional[Tuple[str]] = None
#     edges_exclude: Tuple[str] = tuple()
#     params: Tuple[str, str, str] = ('base', 'free', 'random')
#     period: float = 1.0

@dataclass
class ScenarioConfig:
    name: str = 'intersection'
    seed_offset: int = 0
    inverse_seeding: bool = False
    net_path: Optional[str] = None
    route_path: Optional[str] = None
    add_path: Optional[str] = None
    num_road_scenarios: int = 2147483647
    num_traffic_scenarios: int = 2147483647
    vType_rnd: bool = False
    vType_rnd_num: int = 100
    vType_rnd_path: str = abspath(
        join(
            dirname(__file__),
            "..",
            "..",
            "tools",
            "createVehTypeDistribution.py",
        )
    )
    vType_rnd_config_path: str = abspath(
        join(
            dirname(__file__),
            "..",
            "data",
            "vType_distribution",
            "EIDM.txt",
        )
    )
    ego_init: bool = False
    ego_init_freeze: float = 0.0
    ego_from_edges: Optional[Tuple[str]] = None
    ego_to_edges: Optional[Tuple[str]] = None
    ego_init_position: Union[str, float] = '5.0'
    ego_init_lane: Union[str, float] = 'free'
    ego_init_speed: Union[str, float] = 'random'
    traffic_init: bool = False
    traffic_init_edges: Optional[Tuple[str]] = None
    traffic_init_edges_exclude: Optional[Tuple[str]] = None
    traffic_init_edges_exclude_ego: bool = True
    traffic_init_spread: float = 10.0
    traffic_init_params: Tuple[str, str, str] = ('random_free', 'best', 'random')  # departPos, departLane, departSpeed
    traffic_spawn: bool = False
    traffic_spawn_edges: Optional[Tuple[str]] = None
    traffic_spawn_edges_exclude: Tuple[str] = tuple()
    traffic_spawn_params: Tuple[str, str, str] = ('base', 'free', 'random')
    traffic_spawn_period: float = 1.0
    traffic_vTypes: Optional[Tuple[str]] = None
    traffic_scale: Tuple[float, float] = tuple([0.4, 0.8])
    ego_vType: Optional[str] = None
    generation_num_threads: int = 8
    generation_num_buffer: int = 20

@dataclass
class VehicleConfig:
    dynamics_model: DynamicsModel = MISSING
    car_model: CarModel = CarModel.BMW320i  # Defines which vehicle parameters to use
    v_min: float = 0.0  # Max and min velocity, 50 km/h default
    v_max: float = 13.889
    accel_max: float = 3.5  # m/s2 acceleration and deceleration
    decel_max: float = -5.0
    length: float = 4.2  # length and width of vehicle
    width: float = 1.6
    caster_effect: bool = False  # See driver_dojo.vehicles.TUMVehicle.control(). We only use this for the manual_control.py.
    course_resolution: float = 0.1

@dataclass
class DebugConfig:
    debug: bool = False
    visdom_host: str = 'localhost'
    visdom_port: int = 8097
    visdom_log_path: Optional[str] = None  # Will log visdom state to file, replay with `visdom_obj.replay(log_path)`
    visdom_split_episodes: bool = False
    step_period: int = 30
    episode_period: int = 1
    verbose: bool = False

@dataclass
class ActionConfig:
    space: ActionSpace = MISSING
    extended_road_options: bool = False
    cont_normalized_actions: bool = True
    disc_dimensions: Tuple[int, int] = (5, 5) # Discretized action space: throttle and steering num discretization
    hie_num_target_speeds: int = 5  # Hierarchical: number of target speeds
    hie_accel: bool = False
    disc_hie_cross_prod: bool = False  # Discretized/Hierarchical: Cross-product action space? See code.


# @dataclass
# class VariationConfig:
#     network: Optional[Tuple[GeneratableRoad]] = None
#     network_list_compose: bool = False
#     network_netgenerate: bool = False
#     netgenerate_xml_path: str = ""
#     traffic_routing: bool = False  # The new traffic routing mechanism
#     traffic_routing_period: Optional[
#         float
#     ] = None  # 1/period insertions per second. If None -> One vehicle per simulation step.
#     traffic_routing_start_edges: Optional[Tuple[str]] = None  # On which edges to insert
#     traffic_routing_goal_strategy: Optional[
#         str
#     ] = None  # Which goal strategy to follow, i.e. how to select goal edge based from start edge
#     traffic_routing_randomTrips: bool = (
#         False  # Generate traffic through SUMO's randomTrips.py tooling script
#     )
#     randomTrips_py_path: str = os.path.abspath(
#         pjoin(os.path.dirname(__file__), "..", "..", "tools", "randomTrips.py")
#     )
#     randomTrips_args: Optional[str] = None
#     randomTrips_period: float = 0.1
#     traffic_vTypes: bool = True
#     vTypeDistribution_py_path: str = os.path.abspath(
#         pjoin(
#             os.path.dirname(__file__),
#             "..",
#             "..",
#             "tools",
#             "createVehTypeDistribution.py",
#         )
#     )
#     vTypeDistribution_config_path: str = os.path.abspath(
#         pjoin(
#             os.path.dirname(__file__),
#             "..",
#             "data",
#             "vType_distribution",
#             "EIDM.txt",
#         )
#     )
#     vTypeDistribution_num: int = 200
#     vTypeDistribution_args: Optional[str] = None
#     emergency_break: bool = False
#     emergency_break_interval: float = 10.0
#     traffic_speed: bool = False
#     traffic_speed_interval: float = 2.0
#     traffic_speed_mult_range: Tuple[float, float] = (0.7, 1.3)
#     traffic_density_range: Tuple[float, float] = (0.3, 1.0)


@dataclass
class CarlaObserverConfig:
    sensor_type: CarlaSensor = MISSING
    width: int = 1024
    height: int = 1024
    location: Tuple[float, float, float] = (0.5, 0.0, 1.7)  # Relative transform w.r.t ego
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # See: https://carla.readthedocs.io/en/latest/ref_sensors/ (Blueprint attributes)
    attributes: Dict[str, Union[float, int, str]] = field(
        default_factory=lambda: {
            'fov': 90
        }
    )

@dataclass
class ObservationConfig:
    observers: List[Observer] = field(
        default_factory=lambda: [
            Observer.EgoVehicle,
            Observer.RadiusVehicle,
            #Observer.AvailableOptions,
            #Observer.BirdsEye,
            #Observer.Waypoints,
            Observer.SubGoals,
            Observer.RoadShape,
        ]
    )
    # carla_observers: List[CarlaObserverConfig] = field(
    #     default_factory=lambda: [
    #         CarlaObserverConfig(CarlaSensor.RGBCamera),
    #         CarlaObserverConfig(CarlaSensor.DepthCamera)
    #     ]
    # )
    carla_observers: List[CarlaObserverConfig] = field(
        default_factory=lambda: []
    )
    feature_scaling: Optional[FeatureScaling] = FeatureScaling.Standardize
    cwh: bool = True  # Should image obs be presented in CWH format. Else, WHC.
    relative_to_ego: bool = True  # Compute x, y and angles relative to ego
    num_frame_stack: int = 2  # Stack observations. 1 => no stacking
    rvo_radius: float = 70.0  # RadiusVehicleObserver
    rvo_num_vehicles: int = 15
    rvo_speed_range: Tuple[float, float] = (0, 19.444)
    rvo_accel_range: Tuple[int, int] = (-5, 5)
    rvo_signals: bool = True
    rvo_context: bool = True
    wp_num: int = 5
    wp_sampling_dist: float = 4.0
    sub_goal_num: int = 3  # Number of next sub-goals to present to the agent
    sub_goal_dist_max: float = 50.0
    rs_num_rays: int = 20  # How many rays to cast for RoadObserver
    rs_opening_angle: int = 180  # Opening angle into vehicle heading direction
    rs_num_inter_per_ray: int = 3  # Return the n nearest intersection points per ray
    rs_ray_dist: float = 50.0  # Length of a ray-cast


@dataclass
class NavigationConfig:
    step_size: float = 2.0  # Distance between waypoints
    num_waypoints: int = 10  # Number of waypoints to sample per step
    initial_dist: float = 5.0  # Distance of first waypoint measured from front bumper
    only_end_goal: bool = False  # Just use end-goal as goal
    sub_goal_consume_dist: float = 6.0  # Distance at which goals are consumed


@dataclass
class RewardConfig:
    timeout_penalty: float = -10.0
    driving_circles_penalty: float = -10.0
    collision_penalty: float = -10.0
    stand_still_penalty: float = 0.0
    goal_reward: float = 10.0
    sub_goal_reward: float = 5.0
    off_route_penalty: float = -10.0
    speed_reward: float = 1.0
    constant_reward: float = 0.0


@dataclass
class Config:
    simulation: SimulationConfig = SimulationConfig()
    rendering: RenderingConfig = RenderingConfig()
    scenario: ScenarioConfig = ScenarioConfig()
    actions: ActionConfig = ActionConfig()
    observations: ObservationConfig = ObservationConfig()
    vehicle: VehicleConfig = VehicleConfig()
    navigation: NavigationConfig = NavigationConfig()
    #variation: VariationConfig = VariationConfig()
    reward: RewardConfig = RewardConfig()
    debug: DebugConfig = DebugConfig()

    def __post_init__(self):
        if not self.actions.space == ActionSpace.Semantic:  # These observers only make sense for the semantic action space
            assert Observer.Waypoints not in self.observations.observers
            assert Observer.AvailableOptions not in self.observations.observers
