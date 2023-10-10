from abc import ABC, abstractmethod
from dataclasses import fields
from typing import List, Union

from driver_dojo.vehicle.attachments import WaypointAttachment, SubGoalAttachment, TrafficRandomizerAttachment


class BaseVehicle(ABC):
    # Note: Has to be reset after TrafficManager, due to state being polled from TM in reset()
    def __init__(self, config, traffic_manager):
        self.config = config
        self.traffic_manager = traffic_manager
        self._state = None
        self._attachments: List[Union[WaypointAttachment, SubGoalAttachment, TrafficRandomizerAttachment]] = list()
        self._veh_config = self.config.vehicle
        self._veh_id = self.config.simulation.egoID

    def reset(self):
        self._state = self.traffic_manager.get_actor_state(self._veh_id)

        # Broadcast vehicle config to SUMO
        self._state.length = self._veh_config.length
        self._state.width = self._veh_config.width
        self._state.velocity_min = self._veh_config.v_max  #self._state.velocity_min = self._veh_config.v_min sadly not available in SUMO
        self._state.velocity_max = self._veh_config.v_max  #self._state.velocity_min = self._veh_config.v_min sadly not available in SUMO
        self._state.accel_max = self._veh_config.accel_max
        self._state.decel_max = self._veh_config.decel_max
        # self._state.velocity = self._veh_config.start_velocity if self._veh_config.start_velocity is not None else self._state.velocity  # TODO: randomization
        # TODO: start_offset, start_lane_offset -> position -> broadcast via TraCI

        self.traffic_manager.set_actor_state(self._state)
        self._step_attachments(reset=True)
        self._state_to_properties()

    @abstractmethod
    def control(self, x, y):
        return

    def attach(self, x):
        self._attachments.append(x)

    @property
    def attachments(self):
        return self._attachments

    def get_attachment(self, attach_cls):
        for a in self._attachments:
            if isinstance(a, attach_cls):
                return a
        raise ValueError(f"No attachment of type {type}!")

    def has_attachment(self, attach_cls):
        for a in self._attachments:
            if isinstance(a, attach_cls):
                return True
        return False

    def _step_attachments(self, reset=False):
        for a in self._attachments:
            if reset:
                a.reset()
            a.step()

    def _state_to_properties(self):
        for field in fields(self._state):
            attr_name = field.name
            if attr_name == 'sumo_repr':
                continue

            setattr(self, attr_name, getattr(self._state, attr_name))

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, x):
        self._state = self.traffic_manager.set_actor_state(x)
        self._state_to_properties()
        self._step_attachments()

    @property
    def waypoints(self):
        return self.waypoint_attachment.waypoints

    @property
    def sub_goals(self):
        return self.sub_goal_attachment.sub_goals

    @property
    def waypoint_attachment(self):
        assert True in [isinstance(x, WaypointAttachment) for x in self._attachments], "Attach a WaypointManager first!"
        wm = self.get_attachment(WaypointAttachment)
        return wm

    @property
    def sub_goal_attachment(self):
        assert True in [isinstance(x, SubGoalAttachment) for x in self._attachments], "Attach a SubGoalManager first!"
        sgm = self.get_attachment(SubGoalAttachment)
        return sgm
