from driver_dojo.core.types import RoadOptionsExtended, RoadOptions
import driver_dojo.common.runtime_vars as runtime_vars
from abc import ABC, abstractmethod


class BaseActions(ABC):
    def __init__(self):
        self._action_space = None
        self.RoadOptions = (
            RoadOptionsExtended
            if runtime_vars.config["actions"]["extended_road_options"]
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
    def space(self):
        pass
