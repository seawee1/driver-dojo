from abc import ABC, abstractmethod

from gym.spaces import Box
import numpy as np

from driver_dojo.core.types import FeatureScaling


class BaseObserver(ABC):
    def __init__(self, config, vehicle, traffic_manager):
        self.config = config
        self.vehicle = vehicle
        self.traffic_manager = traffic_manager
        self._observation_space = None
        self._obs_config = self.config.observations
        self._scenario = None
        self.low = None
        self.high = None
        self.inf_mask = None

    def reset(self):
        return

    @abstractmethod
    def step(self):
        return

    @abstractmethod
    def explain(self):
        return

    def _normalize_obs(self, obs):
        if not isinstance(obs, np.ndarray):
            obs = np.array(obs, dtype=np.float64)

        if self.inf_mask is None:  # We only need to do this once
            self.inf_mask = np.logical_and(self.low != -np.inf, self.high != np.inf)

        if self._obs_config.feature_scaling is None:
            return obs

        obs[self.inf_mask] = (obs[self.inf_mask] - self.low[self.inf_mask]) / (self.high[self.inf_mask] - self.low[self.inf_mask])
        if self._obs_config.feature_scaling == FeatureScaling.Standardize:
            obs[self.inf_mask] = 2.0 * obs[self.inf_mask] - 1.0

        return obs

    def _create_observation_space(self, low=None, high=None, feature_scaling=None):
        low = self.low if low is None else low
        high = self.low if high is None else high

        if self.config.observations.inf_fix:  # TODO
            low = np.ones_like(low) * -np.inf
            high = np.ones_like(high) * np.inf

        feature_scaling = self._obs_config.feature_scaling if feature_scaling is None else feature_scaling
        assert low is not None and high is not None

        if feature_scaling == FeatureScaling.Standardize:
            a = np.ones_like(low)
            a[self.inf_mask] = np.inf
            obs_space = Box(
                low=-a, high=a, dtype=np.float64
            )
        elif feature_scaling == FeatureScaling.Normalize:
            a = np.zeros_like(low)
            b = np.ones_like(high)
            a[self.inf_mask] = -np.inf
            b[self.inf_mask] = np.inf
            obs_space = Box(
                low=a, high=b, dtype=np.float64
            )
        else:
            obs_space = Box(low=low, high=high, dtype=np.float64)
        return obs_space

    @property
    def observation_space(self):
        if self._observation_space is None:
            self._observation_space = self._create_observation_space()
        return self._observation_space

    @observation_space.setter
    def observation_space(self, x):
        self._observation_space = x
