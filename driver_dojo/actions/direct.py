import numpy as np
from gym.spaces import Box, Discrete

from driver_dojo.actions import BaseActions
from driver_dojo.core.types import ActionSpace


class DirectActions(BaseActions):
    def __init__(self, config, vehicle):
        super().__init__(config, vehicle)
        self._accel_map = None
        self._steering_map = None
        self._max_action = None
        self._min_action = None
        self._action_space = None

    def reset(self):
        pass

    def step(self, action):
        if self._actions_config.space == ActionSpace.Continuous:
            if self._actions_config.cont_normalized_actions:
                # Denormalize continuous action vector
                action = (
                    np.array(action) if not isinstance(action, np.ndarray) else action
                )
                action = (
                    ((action + 1.0) / 2.0) * (self._max_action - self._min_action)
                ) + self._min_action
        elif self.config["actions"]["space"] == ActionSpace.Discretized:
            if self.config["actions"]["disc_hie_cross_prod"]:
                if action == self.action_space.n - 1:
                    action = (0.0, 0.0)
                else:
                    # Discrete action map to continuous action vector
                    # a_0 -> (s_0, ac_0), a_1 -> (s_0, ac_1) ... a_n -> (s_n, ac_0), a_{n+1} -> (s_n, ac_1), ...
                    action_steering = (
                        action
                        // self.config["actions"]["disc_dimensions"][0]
                    )
                    action_accel = (
                        action % self.config["actions"]["disc_dimensions"][1]
                    )
                    action = np.array(
                        [
                            self._steering_map[action_steering],
                            self._accel_map[action_accel],
                        ]
                    )
            else:
                if action == self.action_space.n - 1:
                    action = (0.0, 0.0)
                else:
                    # Discrete action map to continuous action vector
                    # a_0 -> (s_0, 0.0), a_1 -> (s_1, 0.0), ...  a_n -> (s_n, 0.0), a_{n+1} ->(0.0, ac_0), ...
                    action_steering = (
                        self._steering_map[action]
                        if action
                        < self.config["actions"]["disc_dimensions"][0]
                        else 0.0
                    )
                    action_accel = (
                        self._accel_map[
                            action
                            - self.config["actions"]["disc_dimensions"][0]
                        ]
                        if action
                        >= self.config["actions"]["disc_dimensions"][0]
                        else 0.0
                    )
                    action = np.array([action_steering, action_accel])

        # Execute control on vehicle
        self.vehicle.control(action[0], action[1])

    @property
    def action_space(self):
        if self._action_space is not None:
            return self._action_space

        # Define action space
        if self.config["actions"]["space"] == ActionSpace.Continuous:
            veh = self.vehicle
            if self.config["actions"]["cont_normalized_actions"]:
                self._action_space = Box(
                    low=np.array([-1.0, -1.0]),
                    high=np.array([1.0, 1.0]),
                    dtype=np.float64,
                )
                self._min_action = np.array(
                    [
                        veh.dynamics_params.steering.v_min,
                        -veh.dynamics_params.longitudinal.a_max,
                    ]
                )
                self._max_action = np.array(
                    [
                        veh.dynamics_params.steering.v_max,
                        veh.dynamics_params.longitudinal.a_max,
                    ]
                )
            else:
                self._action_space = Box(
                    low=np.array(
                        [
                            veh.dynamics_params.steering.v_min,
                            -veh.dynamics_params.longitudinal.a_max,
                        ]
                    ),
                    high=np.array(
                        [
                            veh.dynamics_params.steering.v_max,
                            veh.dynamics_params.longitudinal.a_max,
                        ]
                    ),
                    dtype=np.float64,
                )
        elif self.config.actions.space == ActionSpace.Discretized:
            veh = self.vehicle
            veh_config = self.config.vehicle
            self._steering_map = np.linspace(
                veh.dynamics_params.steering.v_min,
                veh.dynamics_params.steering.v_max,
                self.config.actions.disc_dimensions[0],
                endpoint=True,
            )
            self._accel_map = np.linspace(
                veh_config.accel_max,
                veh_config.decel_max,
                self.config.actions.disc_dimensions[1],
                endpoint=True,
            )

            if self.config["actions"]["disc_hie_cross_prod"]:
                self._action_space = Discrete(
                    self.config["actions"]["disc_dimensions"][0]
                    * self.config["actions"]["disc_dimensions"][1]
                    + 1
                )
            else:
                self._action_space = Discrete(
                    self.config["actions"]["disc_dimensions"][0]
                    + self.config["actions"]["disc_dimensions"][1]
                    + 1
                )

        return self._action_space

