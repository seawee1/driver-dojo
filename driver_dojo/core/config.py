import os
from omegaconf import MISSING
from os.path import join as pjoin
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

from driver_dojo.core.types import (
    CarFollowingModel,
    DynamicsModel,
    CarModel,
    GeneratableRoad,
    FeatureScaling,
    Observer,
    ActionSpace,
)


@dataclass
class SimulationConfig:
    dt: float = 0.2  # Duration of one environment step
    steps_per_action: int = 1  # How many time steps to repeat one action
    max_time_steps: int = 300  # Maximum amount of environment steps
    stand_still_timeout_after: int = max_time_steps  # Return done after x time steps without ego movement
    net_path: Optional[str] = None  # Path to net.xml
    route_path: Optional[str] = None  # Path to rou.xml
    add_path: Optional[str] = None  # Path to additional, separated by ,
    gui_xml_path: str = os.path.abspath(
        pjoin(os.path.dirname(__file__), "..", "data", "gui.xml")
    )  # GUI runtime_vars xml path
    egoID: str = "ego"  # ID of ego vehicle
    routeID: str = "route_ego"  # ID of ego route
    demand_scale: float = 1.0  # Scale traffic amount. Alert! Don't set bigger than 1.0, as this will create multiple ego spawns!
    render: bool = False  # Set to true for rendering
    render_track_ego: bool = False  # Follow ego vehicle
    render_navigation: bool = False  # Render waypoints and sub-goals
    render_record: Optional[str] = None  # Will take screenshots and store them under 'render_record_images' with subfolders for every episode
    render_record_video: bool = False  # Will convert images into video. Requires ffmpeg
    seed: int = 1337  # The base seed. One to rule them all
    seed_num_maps: Optional[int] = None
    seed_num_traffic: Optional[int] = None
    tls_off: bool = False  # If traffic lights are present, turn them all off
    traffic_manager_radius: float = 100.0  # How many meters' radius to keep track of traffic state around ego vehicle
    filter_traffic: bool = False  # Filter observable traffic down to junction foes and ego followers and leaders
    time_to_impatience: float = 10.0  # How many seconds for non-ego driver to become impatient
    car_following_model: CarFollowingModel = CarFollowingModel.EIDM
    lateral_resolution: float = 0.72
    junction_ignore_ego: bool = False
    sumo_cmd_extra_args: str = ""  # TODO: Add to code.
    subgoals_only_after: bool = True  # If subgoals should only be added after an edge.
    init_time: float = 0.0  # How many seconds to freeze/delay spawn of the ego vehicle and initialize traffic (delay in the case of init_ego=True)
    init_traffic: bool = False  # Initialize the network with traffic via traci.vehicle.add()
    init_traffic_spread: float = 20.0  # Mean spread between spawned traffic vehicles per lane.
    init_traffic_goal_strategy: Optional[str] = False  # How to select goal edge for spawned traffic
    init_traffic_exclude_edges: Optional[List[str]] = None
    init_ego: bool = False  # If ego spawning should be handled through traci.vehicle.add(). Alert: This might cause problems ego routes defined through the rou.xml file.
    init_ego_start_edge: Optional[str] = None
    init_ego_goal_edge: Optional[str] = None
    init_ego_strategy: Optional[str] = None
    info: bool = False  # Info logging activate
    store_for_plotting: Optional[
        str
    ] = None  # Will copy every map (.net.xml and .xodr file) to the folder specified by 'store_for_plotting'
    time_to_teleport: Optional[float] = -1


@dataclass
class VehicleConfig:
    dynamics_model: DynamicsModel = MISSING
    car_model: CarModel = CarModel.BMW320i  # Defines which vehicle parameters to use
    start_offset: Optional[
        float
    ] = None  # Longitudinal road offset on start edge. If None -> random
    start_velocity: Optional[
        float
    ] = None  # Start velocity of vehicle. If None -> random
    lane_offset: Optional[int] = None  # Which lane to start on. If none -> random
    v_min: Optional[float] = 0.0  # Max and min velocity, 50 km/h default
    v_max: Optional[float] = 13.889
    accel: Optional[float] = 3.0  # m/s2 acceleration and deceleration
    decel: Optional[float] = -4.0
    length: Optional[float] = None  # length and width of vehicle
    width: Optional[float] = None
    caster_effect: bool = False  # See driver_dojo.vehicles.TUMVehicle.control(). We only use this for the manual_control.py.
    use_sumo_vehicle_parameters: bool = False  # Take vehicle parameters from vType of ego vehicle


@dataclass
class ActionConfig:
    space: ActionSpace = MISSING
    extended_road_options: bool = False  # Include left and right junction action into action set
    cont_normalized_actions: bool = True  # Continuous action space: normalized [-1, 1] action as input
    disc_dimensions: Tuple[int, int] = (
        5,
        5,
    )  # Discretized action space: throttle and steering num discretization
    hie_num_target_speeds: int = 5  # Hierarchical: number of target speeds
    hie_accel: bool = False  # Hierarchical: Use accel/decel action instead of target speed action?
    hie_use_controller: bool = False  # Hierarchical: Use low-level controller
    disc_hie_cross_prod: bool = False  # Discretized/Hierarchical: Cross-product action space? See code.


@dataclass
class VariationConfig:
    network: Optional[List[GeneratableRoad]] = None
    network_list_compose: bool = False
    network_netgenerate: bool = False
    netgenerate_xml_path: str = ""
    traffic_routing: bool = False  # The new traffic routing mechanism
    traffic_routing_period: Optional[
        float
    ] = None  # 1/period insertions per second. If None -> One vehicle per simulation step.
    traffic_routing_start_edges: Optional[
        List[str]
    ] = None  # On which edges to insert
    traffic_routing_goal_strategy: Optional[str] = None  # Which goal strategy to follow, i.e. how to select goal edge based from start edge
    traffic_routing_randomTrips: bool = False  # Generate traffic through SUMO's randomTrips.py tooling script
    randomTrips_py_path: str = os.path.abspath(
        pjoin(os.path.dirname(__file__), "..", "..", "tools", "randomTrips.py")
    )
    randomTrips_args: Optional[str] = None
    randomTrips_period: float = 0.1
    traffic_vTypes: bool = True
    vTypeDistribution_py_path: str = os.path.abspath(
        pjoin(
            os.path.dirname(__file__),
            "..",
            "..",
            "tools",
            "createVehTypeDistribution.py",
        )
    )
    vTypeDistribution_config_path: str = os.path.abspath(
        pjoin(
            os.path.dirname(__file__), "..", "data", "vType_distribution", "EIDM.txt",
        )
    )
    vTypeDistribution_num: int = 200
    vTypeDistribution_args: Optional[str] = None
    emergency_break: bool = False
    emergency_break_interval: float = 10.0
    traffic_speed: bool = False
    traffic_speed_interval: float = 2.0
    traffic_speed_mult_range: Tuple[float, float] = (0.7, 1.3)


@dataclass
class ObservationConfig:
    observers: List[Observer] = field(
        default_factory=lambda: [
            Observer.EgoVehicle,
            Observer.RadiusVehicle,
            Observer.SubGoals,
            Observer.RoadShape,
            # Observer.BirdsEye,
        ]
    )
    feature_scaling: FeatureScaling = FeatureScaling.Standardize
    cwh: bool = True  # Should image obs be presented in CWH format. Else, WHC.
    relative_to_ego: bool = True  # Compute x, y and angles relative to ego
    num_frame_stack: int = 1  # Stack observations. 1 => no stacking
    rvo_radius: float = 70.0  # RadiusVehicleObserver
    rvo_num_vehicles: int = 15
    rvo_speed_range: Tuple[float, float] = (0, 19.444)
    rvo_accel_range: Tuple[int, int] = (-5, 5)
    rvo_signals: bool = True
    beo_size_meters: int = 32  # BirdEyeObserver diameter in meters, pixels per meter, cwh channel ordering
    beo_ppm: int = 4
    beo_draw_subgoals: bool = True
    wp_num: int = 10  # WaypointObserver num waypoints, x,y range and dist_max for normalization
    wp_xy_range: Tuple[float, float, float, float] = (-50.0, -50.0, 50.0, 50.0)
    wp_dist_max: float = 50.0
    sub_goal_num: int = 3  # Number of next sub_goals to observe, x_yrange ...
    sub_goal_xy_range: Tuple[float, float, float, float] = (-50.0, -50.0, 50.0, 50.0)
    sub_goal_dist_max: float = 50.0
    rs_num_rays: int = 30  # How many rays to cast for RoadObserver
    rs_opening_angle: int = 216  # Opening angle into vehicle heading direction
    rs_num_inter_per_ray: int = 3  # Return the n nearest intersection points per ray
    rs_ray_dist: float = 150.0  # Length of a ray-cast


@dataclass
class NavigationConfig:
    step_size: float = 2.0  # Distance between waypoints
    max_distance: float = 50.0  # How many meters away from ego waypoints are still sampled
    wp_consume_dist: float = 3.0  # At which distance waypoints are consumed
    only_end_goal: bool = False  # Just use end-goal as goal
    sub_goal_consume_dist: float = 10.0  # Distance at which goals are consumed


@dataclass
class RewardConfig:
    timeout_penalty: float = -10.0
    collision_penalty: float = -10.0
    stand_still_penalty: float = 0.0
    goal_reward: float = 10.0
    sub_goal_reward: float = 5.0
    off_route_penalty: float = -10.0
    speed_reward: float = 1.0
    constant_reward: float = 0.0


@dataclass
class DebugConfig:
    plot_road_shape_observations: Optional[
        int
    ] = None  # Use value to plot every x timesteps. You will have to close the first window.
    plot_birds_eye_observations: Optional[int] = None  # Same for BEO


@dataclass
class Config:
    simulation: SimulationConfig = SimulationConfig()
    actions: ActionConfig = ActionConfig()
    observations: ObservationConfig = ObservationConfig()
    vehicle: VehicleConfig = VehicleConfig()
    navigation: NavigationConfig = NavigationConfig()
    variation: VariationConfig = VariationConfig()
    reward: RewardConfig = RewardConfig()
    debug: DebugConfig = DebugConfig()
