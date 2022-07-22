import numpy as np
from gym.spaces import Box, Discrete

from driver_dojo.actions.base_actions import BaseActions
import driver_dojo.common.runtime_vars as runtime_vars
from driver_dojo.core.types import ActionSpace


class DirectActions(BaseActions):
    def reset(self):
        pass

    def step(self, action):
        if runtime_vars.config["actions"]["space"] == ActionSpace.Continuous:
            if runtime_vars.config["actions"]["cont_normalized_actions"]:
                # Denormalize continuous action vector
                action = (
                    np.array(action) if not isinstance(action, np.ndarray) else action
                )
                action = (
                    ((action + 1.0) / 2.0) * (self.max_action - self.min_action)
                ) + self.min_action
        elif runtime_vars.config["actions"]["space"] == ActionSpace.Discretized:
            if runtime_vars.config["actions"]["disc_hie_cross_prod"]:
                num_actions = (
                    runtime_vars.config["actions"]["disc_dimensions"][0]
                    * runtime_vars.config["actions"]["disc_dimensions"][1]
                ) + 1
                if action == num_actions - 1:
                    action = (0.0, 0.0)
                else:
                    # Discrete action map to continuous action vector
                    # a_0 -> (s_0, ac_0), a_1 -> (s_0, ac_1) ... a_n -> (s_n, ac_0), a_{n+1} -> (s_n, ac_1), ...
                    action_steering = (
                        action
                        // runtime_vars.config["actions"]["disc_dimensions"][0]
                    )
                    action_accel = (
                        action % runtime_vars.config["actions"]["disc_dimensions"][1]
                    )
                    action = np.array(
                        [
                            self.steering_map[action_steering],
                            self.accel_map[action_accel],
                        ]
                    )
            else:
                num_actions = (
                    runtime_vars.config["actions"]["disc_dimensions"][0]
                    + runtime_vars.config["actions"]["disc_dimensions"][1]
                ) + 1
                if action == num_actions - 1:
                    action = (0.0, 0.0)
                else:
                    # Discrete action map to continuous action vector
                    # a_0 -> (s_0, 0.0), a_1 -> (s_1, 0.0), ...  a_n -> (s_n, 0.0), a_{n+1} ->(0.0, ac_0), ...
                    action_steering = (
                        self.steering_map[action]
                        if action
                        < runtime_vars.config["actions"]["disc_dimensions"][0]
                        else 0.0
                    )
                    action_accel = (
                        self.accel_map[
                            action
                            - runtime_vars.config["actions"]["disc_dimensions"][0]
                        ]
                        if action
                        >= runtime_vars.config["actions"]["disc_dimensions"][0]
                        else 0.0
                    )
                    action = np.array([action_steering, action_accel])

        # Execute control on vehicle
        runtime_vars.vehicle.control(action[0], action[1])

    @property
    def space(self):
        # Define action space
        if runtime_vars.config["actions"]["space"] == ActionSpace.Continuous:
            if runtime_vars.config["actions"]["cont_normalized_actions"]:
                self._action_space = Box(
                    low=np.array([-1.0, -1.0]),
                    high=np.array([1.0, 1.0]),
                    dtype=np.float64,
                )
                self.min_action = np.array(
                    [
                        runtime_vars.vehicle.tum_params.steering.v_min,
                        -runtime_vars.vehicle.tum_params.longitudinal.a_max,
                    ]
                )
                self.max_action = np.array(
                    [
                        runtime_vars.vehicle.tum_params.steering.v_max,
                        runtime_vars.vehicle.tum_params.longitudinal.a_max,
                    ]
                )
            else:
                self._action_space = Box(
                    low=np.array(
                        [
                            runtime_vars.vehicle.tum_params.steering.v_min,
                            -runtime_vars.vehicle.tum_params.longitudinal.a_max,
                        ]
                    ),
                    high=np.array(
                        [
                            runtime_vars.vehicle.tum_params.steering.v_max,
                            runtime_vars.vehicle.tum_params.longitudinal.a_max,
                        ]
                    ),
                    dtype=np.float64,
                )
        elif runtime_vars.config["actions"]["space"] == ActionSpace.Discretized:
            self.steering_map = np.linspace(
                runtime_vars.vehicle.tum_params.steering.v_min,
                runtime_vars.vehicle.tum_params.steering.v_max,
                runtime_vars.config["actions"]["disc_dimensions"][0],
                endpoint=True,
            )
            self.accel_map = np.linspace(
                runtime_vars.vehicle.decel,
                runtime_vars.vehicle.accel,
                runtime_vars.config["actions"]["disc_dimensions"][1],
                endpoint=True,
            )

            if runtime_vars.config["actions"]["disc_hie_cross_prod"]:
                self._action_space = Discrete(
                    runtime_vars.config["actions"]["disc_dimensions"][0]
                    * runtime_vars.config["actions"]["disc_dimensions"][1]
                    + 1
                )
            else:
                self._action_space = Discrete(
                    runtime_vars.config["actions"]["disc_dimensions"][0]
                    + runtime_vars.config["actions"]["disc_dimensions"][1]
                    + 1
                )

        return self._action_space
