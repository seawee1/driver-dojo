from abc import ABC

import gym
import numpy as np

from driver_dojo.observer import BaseObserver


class MultiObserver(BaseObserver):
    """ Combine multiple observer into one. If multiple vector observer are provided, MultiObserver returns a
    combined vector observation space. If multiple image observer are provided, MultiObserver returns a combined
    image observation space. If a mixture of vector and image observer are provided, MultiObserver returns a dict
    observation space with entries 'vector' and 'image'.
    """

    def __init__(self, *args, config, vehicle, traffic_manager):
        super().__init__(config, vehicle, traffic_manager)
        self.obs_members = {"vector": [], "image": []}
        self.obs_spaces = {"vector": None, "image": None}
        self.multi_observer_type = None
        self._combine(args)

    def _combine(self, observers):
        # Sort our observer into 'image' and 'vector' observer buckets
        for observer in observers:
            if len(observer.observation_space.shape) > 1:
                self.obs_members["image"].append(observer)
            else:
                self.obs_members["vector"].append(observer)

        if len(self.obs_members["vector"]) > 0:
            low = np.concatenate(
                [x.low for x in self.obs_members["vector"]]
            )
            high = np.concatenate(
                [x.high for x in self.obs_members["vector"]]
            )
            self.obs_spaces["vector"] = self._create_observation_space(low, high)

        if len(self.obs_members["image"]) > 0:
            low = np.concatenate(
                [x.low for x in self.obs_members["image"]],
                axis=2,
            )
            high = np.concatenate(
                [x.high for x in self.obs_members["image"]],
                axis=2,
            )
            self.obs_spaces["image"] = self._create_observation_space(low, high)

        if self.obs_spaces["image"] is None:
            self.observation_space = self.obs_spaces["vector"]
            self.multi_observer_type = "vector"
        elif self.obs_spaces["vector"] is None:
            self.observation_space = self.obs_spaces["image"]
            self.multi_observer_type = "image"
        else:
            self.observation_space = gym.spaces.Dict(
                dict(
                    vector=self.obs_spaces["vector"],
                    image=self.obs_spaces["image"]
                )
            )
            self.multi_observer_type = "dict"

    def reset(self):
        for observer in self.obs_members['vector'] + self.obs_members['image']:
            observer.reset()
        return self._observe('step')

    def step(self):
        # for observer in self.obs_members['vector']:
        #     obs = observer.step()
        #     obs_wrong = np.logical_or(obs < observer.observation_space.low, obs > observer.observation_space.high)
        #     if np.any(obs_wrong):
        #         print(obs[obs_wrong])
        #         print(type(observer), np.array(observer.explain())[obs_wrong])
        #obs = self._observe('step')
        return self._observe('step')

    def _observe(self, fn_name):
        obs_image = None
        obs_vec = None
        if self.obs_spaces["image"] is not None:
            obs_image = np.concatenate([getattr(observer, fn_name)() for observer in self.obs_members["image"]], axis=2)
        if self.obs_spaces["vector"] is not None:
            obs_vec = np.concatenate([getattr(observer, fn_name)() for observer in self.obs_members["vector"]])

        if obs_image is not None and obs_vec is not None:
            return dict(
                vector=obs_vec,
                image=obs_image,
            )
        elif obs_image is not None:
            return obs_image
        else:
            return obs_vec

    def explain(self):
        return  # TODO: Calls for sub-observers

    def get_observer(self, observer_cls):
        for observer in self.obs_members['vector'] + self.obs_members['image']:
            if isinstance(observer, observer_cls):
                return observer
        raise ValueError

    @property
    def observation_space(self):
        return self._observation_space

    @observation_space.setter
    def observation_space(self, x):
        self._observation_space = x
