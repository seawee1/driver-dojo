import sys
sys.path.append("C:\\Users\\seegras\\Git\\PythonRobotics")  # TODO: This is ugly

from os.path import dirname as dirname
from os.path import join as pjoin
from omegaconf import OmegaConf
from gym.envs.registration import register
from copy import deepcopy
import os

from driver_dojo.core.config import Config
from driver_dojo.core.types import *

#########
# TODOs #
#########
# https://sumo.dlr.de/docs/Simulation/Output/SSM_Device.html#conflict_types
# SSM-device could be used generate safety metrics for evaluation
# ...

###############################
### Environment Definitions ###
###############################
DIRECT_DYN_MODEL = DynamicsModel.KS

cont_config = dict(
    actions=dict(space=ActionSpace.Continuous),
    vehicle=dict(dynamics_model=DIRECT_DYN_MODEL),
)
disc_config = dict(
    actions=dict(space=ActionSpace.Discretized),
    vehicle=dict(dynamics_model=DIRECT_DYN_MODEL),
)
sem_d_config = dict(
    actions=dict(space=ActionSpace.Semantic),
    vehicle=dict(dynamics_model=DIRECT_DYN_MODEL),
)
sem_config = dict(
    actions=dict(space=ActionSpace.Semantic),
    vehicle=dict(dynamics_model=DynamicsModel.God),
)

for action_name, base_config, veh_model in [
    ("Cont", cont_config, DIRECT_DYN_MODEL.value),
    ("Disc", disc_config, DIRECT_DYN_MODEL.value),
    ("Sem", sem_d_config, DIRECT_DYN_MODEL.value),
    ("Sem", sem_config, DynamicsModel.God.value),
]:
    #####################################
    ### Random road network scenarios ###
    #####################################

    ####################
    ### Intersection ###
    ####################
    # Driving these scenarios by hand takes about 15 seconds on average.
    # We give the agent a little extra (in case there is jamming and stuff).
    max_episode_steps = 300

    config = Config()
    config.simulation.max_episode_steps = max_episode_steps
    config.scenario.traffic_init = True
    config.scenario.traffic_init_spread = 30.0
    config.scenario.traffic_spawn = True
    config.scenario.traffic_spawn_period = 1.0
    config.scenario.vType_rnd = False
    config.scenario.ego_init = True
    config.vehicle.v_max = SpeedClass.Urban.value
    for scenario_name in ['Intersection_S_RBL', 'Intersection_S_Major', 'Intersection_S_Minor']:
        config.scenario.name = scenario_name
        env_config = OmegaConf.merge(config, base_config)
        register(
            id=f"DriverDojo/{action_name}-{veh_model}-{scenario_name}-v0",
            entry_point=f"driver_dojo.core.env:DriverDojoEnv",
            kwargs=dict(_config=env_config),
        )
    # for seed_name, num_road, num_traf in seeding:
    #     config.scenario.num_road_scenarios = num_road
    #     config.scenario.num_traffic_scenarios = num_traf
        #env_config = OmegaConf.merge(config, base_config)
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-Intersection-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=env_config),
    # )
    # simulation_config = dict(
    #     max_time_steps=max_episode_steps,
    #     init_time=0.0,
    #     init_traffic=True,
    #     init_traffic_spread=30.0,
    #     init_traffic_goal_strategy="intersection",
    #     init_traffic_exclude_edges=[0],
    #     init_ego=True,
    #     init_ego_start_edge=0,
    #     init_ego_goal_edge=-2,  # This way we always have to drive across the intersection
    #     init_ego_strategy="intersection",
    # )
    # var_config = dict(
    #     network=[GeneratableRoad.Intersection],
    #     vTypeDistribution_config_path=os.path.abspath(
    #         pjoin(dirname(__file__), "data", "vType_distribution", "EIDM.txt")
    #     ),
    #     traffic_routing=True,
    #     traffic_routing_period=0.5,
    #     traffic_routing_start_edges=None,
    #     traffic_routing_goal_strategy="intersection",
    # )
    # config = deepcopy(base_config)
    # config["variation"] = var_config
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Urban.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-Intersection-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    # ##################
    # ### Roundabout ###
    # ##################
    # max_episode_steps = 300
    # simulation_config = dict(
    #     max_time_steps=max_episode_steps,
    #     init_time=0.0,
    #     init_traffic=True,
    #     init_traffic_spread=30.0,
    #     init_traffic_goal_strategy="roundabout",
    #     init_traffic_exclude_edges=['in0'],
    #     init_ego=True,
    #     init_ego_start_edge="in0",
    #     init_ego_goal_edge="out*",  # This way we always have to drive across the intersection
    #     init_ego_strategy="roundabout",
    # )
    # var_config = dict(
    #     network=[GeneratableRoad.Roundabout],
    #     vTypeDistribution_config_path=os.path.abspath(
    #         pjoin(dirname(__file__), "data", "vType_distribution", "EIDM.txt")
    #     ),
    #     traffic_routing=True,
    #     traffic_routing_period=0.5,
    #     traffic_routing_start_edges=None,
    #     traffic_routing_goal_strategy="roundabout",
    # )
    # config = deepcopy(base_config)
    # config["variation"] = var_config
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Urban.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-Roundabout-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    # ###############
    # ### Highway ###
    # ###############
    # # Driving these scenarios by hand takes about 30-40 seconds on average.
    # # We give the agent a little extra.
    # max_episode_steps = 300
    #
    # ###########################
    # ###### Highway Entry ######
    # ###########################
    # simulation_config = dict(
    #     max_time_steps=max_episode_steps,
    #     init_time=0.0,
    #     init_traffic=True,
    #     init_traffic_spread=30.0,
    #     init_traffic_goal_strategy="min-or-0",
    #     init_traffic_exclude_edges=["-1"],  # The entry road
    #     init_ego=True,
    #     init_ego_start_edge="-1",
    #     init_ego_goal_edge="-4",
    # )
    # var_config = dict(
    #     network=[GeneratableRoad.HighwayEntry],
    #     vTypeDistribution_config_path=os.path.abspath(
    #         pjoin(dirname(__file__), "data", "vType_distribution", "EIDM.txt")
    #     ),
    #     traffic_routing=True,
    #     traffic_routing_period=0.4,
    #     traffic_routing_start_edges=["-0", "4"],
    #     traffic_routing_goal_strategy="min-or-0",
    # )
    # config = deepcopy(base_config)
    # config["variation"] = var_config
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Highway.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-HighwayEntry-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    # ###########################
    # ###### Highway Drive ######
    # ###########################
    # simulation_config = dict(
    #     max_time_steps=max_episode_steps,
    #     init_time=0.0,
    #     init_traffic=True,
    #     init_traffic_spread=30.0,
    #     init_traffic_goal_strategy="same",
    #     init_traffic_exclude_edges=[],  # The entry road
    #     init_ego=True,
    #     init_ego_start_edge="-0",
    #     init_ego_goal_edge="",
    # )
    # var_config = dict(
    #     network=[GeneratableRoad.HighwayDrive],
    #     vTypeDistribution_config_path=os.path.abspath(
    #         pjoin(dirname(__file__), "data", "vType_distribution", "EIDM.txt")
    #     ),
    #     traffic_routing=True,
    #     traffic_routing_period=0.4,
    #     traffic_routing_start_edges=["-0", "0"],
    #     traffic_routing_goal_strategy="same",
    # )
    # config = deepcopy(base_config)
    # config['vehicle']['start_offset'] = 10.0
    # config["variation"] = var_config
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Highway.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-HighwayDrive-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    # ###########################
    # ###### Highway Exit #######
    # ###########################
    # simulation_config = dict(
    #     max_time_steps=max_episode_steps,
    #     init_time=0.0,
    #     init_traffic=True,
    #     init_traffic_spread=30.0,
    #     init_traffic_goal_strategy="highway-exit",
    #     init_traffic_exclude_edges=["-2", "-0#3"],
    #     init_ego=True,
    #     init_ego_start_edge="-0#0",
    #     init_ego_goal_edge="-2",
    # )
    # var_config = dict(
    #     network=[GeneratableRoad.HighwayExit],
    #     vTypeDistribution_config_path=os.path.abspath(
    #         pjoin(dirname(__file__), "data", "vType_distribution", "EIDM.txt")
    #     ),
    #     traffic_routing=True,
    #     traffic_routing_period=0.4,
    #     traffic_routing_start_edges=["-0#0", "3"],
    #     traffic_routing_goal_strategy="highway-exit",
    # )
    # config = deepcopy(base_config)
    # config["variation"] = var_config
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Highway.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-HighwayExit-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    #
    #
    # ########################
    # ### Custom Scenarios ###
    # ########################
    # this_path = os.path.dirname(os.path.realpath(__file__))
    #
    # simulation_config = dict(
    #     net_path=pjoin(this_path, "data/scenarios/Nuremberg-NOP/map.net.xml"),
    #     route_path=pjoin(this_path, "data/scenarios/Nuremberg-NOP/fraunhofer-cruise.rou.xml"),
    #     add_path=pjoin(this_path, "data/scenarios/Nuremberg-NOP/map.poly.xml"),
    #     max_time_steps=300,
    #     init_time=20.0,
    # )
    # config = deepcopy(base_config)
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Urban.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-FraunhoferNOP-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    # simulation_config = dict(
    #     net_path=pjoin(this_path, "data/scenarios/Urban/map.net.xml"),
    #     route_path=pjoin(this_path, "data/scenarios/Urban/intersection.rou.xml"),
    #     max_time_steps=300,
    #     init_time=20.0,
    # )
    # config = deepcopy(base_config)
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Urban.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-Urban_Intersection-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    #
    # ########################
    # ### Fraunhofer stuff ###
    # ########################
    # # Demo
    # simulation_config = dict(
    #     net_path=pjoin(this_path, "data/scenarios/Carla/Town03/map.net.xml"),
    #     route_path=pjoin(this_path, "data/scenarios/Carla/Town03/demo.rou.xml"),
    #     max_time_steps=300,
    #     init_time=20.0,
    #     demand_scale=0.5,
    # )
    # config = deepcopy(base_config)
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Urban.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-Carla_Town03_Demo-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )
    #
    # # SafeDQN
    # simulation_config = dict(
    #     net_path=pjoin(this_path, "data/scenarios/Carla/Town03/Town03.net.xml"),
    #     route_path=pjoin(this_path, "data/scenarios/Carla/Town03/Town03ICRA_1.rou.xml"),
    #     max_time_steps=300,
    #     init_time=20.0,
    #     demand_scale=1.0,
    # )
    # config = deepcopy(base_config)
    # config["simulation"] = simulation_config
    # config["vehicle"]["v_max"] = float(SpeedClass.Urban.value)
    # config = OmegaConf.create(config)
    #
    # register(
    #     id=f"DriverDojo/{action_name}-{veh_model}-Carla_Town03_SafeDQN_RightTurn-v0",
    #     entry_point=f"driver_dojo.core.env:DriverDojoEnv",
    #     kwargs=dict(_config=config),
    # )

