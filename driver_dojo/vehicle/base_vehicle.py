from abc import ABC, abstractmethod

from driver_dojo.core.types import CarModel
from driver_dojo.vehicle.vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from driver_dojo.vehicle.vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from driver_dojo.vehicle.vehiclemodels.parameters_vehicle3 import parameters_vehicle3
import driver_dojo.common.runtime_vars as runtime_vars


class BaseVehicle(ABC):
    def __init__(self, config, egoID, traffic_manager, dt):
        self.waypoint_manager = None
        self.sub_goal_manager = None
        self.traffic_randomizer = None
        self.vehicle_config = config
        self.egoID = egoID
        runtime_vars.traffic_manager = traffic_manager
        self.dt = dt

        parameters_vehicle_map = {
            CarModel.FordEscort: parameters_vehicle1,
            CarModel.BMW320i: parameters_vehicle2,
            CarModel.VWVanagon: parameters_vehicle3,
        }
        self.vehicle_config = runtime_vars.config["vehicle"]
        self.tum_params = parameters_vehicle_map[self.vehicle_config["car_model"]]()
        self.sumo_params = runtime_vars.traffic_manager.ego_params

        # Set vehicle accel, decel, v_min, v_max, length and width based on config specifications. We use TUM vehicle
        # params in the standard case, which can be overwritten with SUMO vType params if
        # 'use_sumo_vehicle_parameters' flag is set. Manually specified parameters always override TUM or SUMO
        # parameters.
        if self.vehicle_config["accel"] is None:
            self.accel = self.tum_params.longitudinal.a_max
        if self.vehicle_config["use_sumo_vehicle_parameters"]:
            self.accel = self.sumo_params["accel"]
        if self.vehicle_config["accel"] is not None:
            self.accel = self.vehicle_config["accel"]
        self.tum_params.longitudinal.a_max = self.accel

        if self.vehicle_config["decel"] is None:
            self.decel = -self.tum_params.longitudinal.a_max
        if self.vehicle_config["use_sumo_vehicle_parameters"]:
            self.decel = self.sumo_params["decel"]
        if self.vehicle_config["decel"] is not None:
            self.decel = self.vehicle_config["decel"]
        # TUM models use -longitudinal.a_max for deceleration

        if self.vehicle_config["v_min"] is None:
            self.v_min = self.tum_params.longitudinal.v_min
        if self.vehicle_config["use_sumo_vehicle_parameters"]:
            self.v_min = self.sumo_params["v_min"]
        if self.vehicle_config["v_min"] is not None:
            self.v_min = self.vehicle_config["v_min"]
        self.tum_params.longitudinal.v_min = self.v_min

        if self.vehicle_config["v_max"] is None:
            self.v_max = self.tum_params.longitudinal.v_max
        if self.vehicle_config["use_sumo_vehicle_parameters"]:
            self.v_max = self.sumo_params["v_max"]
        if self.vehicle_config["v_max"] is not None:
            self.v_max = self.vehicle_config["v_max"]
        self.tum_params.longitudinal.v_max = self.vehicle_config["v_max"]

        # Set ego length and width if specified in config.
        if self.vehicle_config["length"] is None:
            self.length = self.tum_params.l
        if self.vehicle_config["use_sumo_vehicle_parameters"]:
            self.length = self.sumo_params["length"]
        if self.vehicle_config["length"] is not None:
            self.length = self.sumo_params["length"]
        self.tum_params.l = self.length

        if self.vehicle_config["width"] is None:
            self.width = self.tum_params.w
        if self.vehicle_config["use_sumo_vehicle_parameters"]:
            self.width = self.sumo_params["width"]
        if self.vehicle_config["width"] is not None:
            self.width = self.vehicle_config["width"]
        self.tum_params.w = self.width

        # Broadcast ego vehicle parameters back to TrafficManager, which does the Traci calls
        self.sumo_params = dict(
            accel=self.accel,
            decel=self.decel,
            v_min=self.v_min,
            v_max=self.v_max,
            length=self.length,
            width=self.width,
        )
        runtime_vars.traffic_manager.ego_params = self.sumo_params

    def reset(self):
        # Broadcast vehicle params
        runtime_vars.traffic_manager.ego_params = self.sumo_params

        # Set initial speed and position
        ego_state_other = runtime_vars.traffic_manager.ego_state_str("laneID")
        runtime_vars.traffic_manager.modify_vehicle_state(
            self.egoID,
            pos=self.vehicle_config["start_offset"],
            speed=self.vehicle_config["start_velocity"],
            laneID=ego_state_other,
            override_signals=True,
        )

    @property
    def edgeID(self):
        return runtime_vars.traffic_manager.ego_state_str("edgeID")

    @property
    def laneID(self):
        return runtime_vars.traffic_manager.ego_state_str("laneID")

    @property
    def laneIndex(self):
        return runtime_vars.traffic_manager.ego_state_str("lane_index")

    @property
    def waypoints(self):
        assert (
            self.waypoint_manager is not None
        ), "Call attach_waypoint_manager(...) first to use this field!"
        return self.waypoint_manager.waypoints

    @property
    def sub_goals(self):
        assert (
            self.sub_goal_manager is not None
        ), "Call attach_sub_goal_manager(...) first to use this field!"
        return self.sub_goal_manager.sub_goals

    def _set_position(self, position):
        runtime_vars.traffic_manager.modify_vehicle_state(self.egoID, xy=position)

    def _set_speed(self, speed):
        runtime_vars.traffic_manager.modify_vehicle_state(self.egoID, speed=speed)

    def _set_angle(self, angle):
        runtime_vars.traffic_manager.modify_vehicle_state(self.egoID, yaw=angle)

    def attach_waypoint_manager(self, waypoint_manager):
        self.waypoint_manager = waypoint_manager

    def attach_sub_goal_manager(self, goal_manager):
        self.sub_goal_manager = goal_manager

    def attach_traffic_randomizer(self, traffic_randomizer):
        self.traffic_randomizer = traffic_randomizer

    @abstractmethod
    def control(self, a, b):
        pass

    @property
    @abstractmethod
    def position(self):
        pass

    @property
    @abstractmethod
    def speed(self):
        pass

    @property
    @abstractmethod
    def acceleration(self):
        pass

    @property
    @abstractmethod
    def steering_angle(self):
        pass

    @property
    @abstractmethod
    def angle(self):
        pass

    @position.setter
    @abstractmethod
    def position(self, position):
        pass

    @speed.setter
    @abstractmethod
    def speed(self, speed):
        pass

    @steering_angle.setter
    @abstractmethod
    def steering_angle(self, steering_angle):
        pass

    @angle.setter
    @abstractmethod
    def angle(self, angle):
        pass
