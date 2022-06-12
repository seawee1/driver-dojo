from abc import ABC, abstractmethod


class BaseObserver(ABC):
    def __init__(self):
        self.observation_space = None

    def reset(self):
        pass

    @abstractmethod
    def observe(self):
        pass

    @property
    def space(self):
        return self.observation_space
