from driver_dojo.core.types import RoadOptionsExtended, RoadOptions, ActionSpace
from abc import ABC, abstractmethod


class BaseActions(ABC):
    def __init__(self, config, vehicle):
        self.config = config
        self.vehicle = vehicle
        self._action_space = None
        self._actions_config = self.config.actions
        self._road_options = (
            RoadOptionsExtended
            if self._actions_config.extended_road_options
            else RoadOptions
        )

    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def step(self, action):
        pass

    @property
    @abstractmethod
    def action_space(self):
        pass

    @property
    def noop_action(self):
        if self.config["actions"]["space"] == ActionSpace.Continuous:
            return [0.0, 0.0]
        else:
            return self.action_space.n - 1
