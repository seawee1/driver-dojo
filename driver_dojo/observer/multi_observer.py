import gym
import numpy as np

from driver_dojo.common.utils import create_observation_space
import driver_dojo.common.runtime_vars as runtime_vars


class MultiObserver:
    """ Combine multiple observer into one. If multiple vector observer are provided, MultiObserver returns a
    combined vector observation space. If multiple image observer are provided, MultiObserver returns a combined
    image observation space. If a mixture of vector and image observer are provided, MultiObserver returns a dict
    observation space with entries 'vector' and 'image'.
    """

    def __init__(self, *args):
        """Creates a new MultiObserver

        Parameters
        ----------
        """

        self.num_frame_stack = runtime_vars.config["observations"]["num_frame_stack"]

        self.observation_members = {"vector": [], "image": []}
        self.observation_spaces = {"vector": None, "image": None}
        self.multi_observer_type = None
        self._combine(args)

    def _combine(self, observers):
        # Sort our observer into 'image' and 'vector' observer buckets
        for observer in observers:
            if len(observer.observation_space.shape) > 1:
                self.observation_members["image"].append(observer)
            else:
                self.observation_members["vector"].append(observer)

        if len(self.observation_members["vector"]) > 0:
            low = np.concatenate(
                [x.low for x in self.observation_members["vector"]]
                * self.num_frame_stack
            )
            high = np.concatenate(
                [x.high for x in self.observation_members["vector"]]
                * self.num_frame_stack
            )
            self.observation_spaces["vector"] = create_observation_space(
                low, high, runtime_vars.config["observations"]["feature_scaling"]
            )
        if len(self.observation_members["image"]) > 0:
            low = np.concatenate(
                [x.low for x in self.observation_members["image"]]
                * self.num_frame_stack,
                axis=2,
            )
            high = np.concatenate(
                [x.high for x in self.observation_members["image"]]
                * self.num_frame_stack,
                axis=2,
            )

            if runtime_vars.config.observations.cwh:
                low = np.rollaxis(low, 2)
                high = np.rollaxis(high, 2)

            self.observation_spaces["image"] = create_observation_space(
                low, high, runtime_vars.config["observations"]["feature_scaling"]
            )

        if self.observation_spaces["image"] is None:
            self.observation_space = self.observation_spaces["vector"]
            self.multi_observer_type = "vector"
        elif self.observation_spaces["vector"] is None:
            self.observation_space = self.observation_spaces["image"]
            self.multi_observer_type = "image"
        else:
            self.observation_space = gym.spaces.Dict(
                {
                    "vector": self.observation_spaces["vector"],
                    "image": self.observation_spaces["image"],
                }
            )
            self.multi_observer_type = "dict"

        self.reset()

    def reset(self):
        # Frame-stacking stuff
        if self.multi_observer_type == "dict":
            self.frames = {
                "vector": np.zeros(self.observation_space["vector"].shape),
                "image": np.zeros(self.observation_space["image"].shape),
            }
        else:
            self.frames = np.zeros(self.observation_space.shape)

        for observer in (
            self.observation_members["vector"] + self.observation_members["image"]
        ):
            observer.reset()

    def step(self):
        def observe_vector(obs, frames):
            shift = obs.shape[0]
            frames = np.roll(frames, shift)
            frames[:shift] = obs
            return frames

        def observe_image(obs, frames):
            shift = obs.shape[0]
            frames = np.roll(frames, shift, axis=0)
            frames[:shift, :, :] = obs
            return frames

        if self.multi_observer_type == "vector":
            self.frames = observe_vector(self._observe(), self.frames)
        elif self.multi_observer_type == "image":
            self.frames = observe_image(self._observe(), self.frames)
        else:
            obs = self._observe()
            self.frames["vector"] = observe_vector(obs["vector"], self.frames["vector"])
            self.frames["image"] = observe_image(obs["image"], self.frames["image"])

        return self.frames

    def _observe(self):
        if self.observation_spaces["image"] is None:
            obs = np.concatenate(
                [observer.observe() for observer in self.observation_members["vector"]]
            )
        elif self.observation_spaces["vector"] is None:
            obs = np.concatenate(
                [observer.observe() for observer in self.observation_members["image"]],
                axis=2,
            )
            if runtime_vars.config.observations.cwh: obs = np.rollaxis(obs, 2)
        else:
            obs = {
                "vector": np.concatenate(
                    [
                        observer.observe()
                        for observer in self.observation_members["vector"]
                    ]
                ),
                "image": np.concatenate(
                    [
                        observer.observe()
                        for observer in self.observation_members["image"]
                    ],
                    2,
                ),
            }
            if runtime_vars.config.observations.cwh: obs['image'] = np.rollaxis(obs['image'], 2)
        return obs

    @property
    def space(self):
        return self.observation_space
