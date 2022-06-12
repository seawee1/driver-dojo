import numpy as np
from gym.spaces import Discrete

from driver_dojo.vehicle.tps_vehicle import TPSVehicle
from driver_dojo.vehicle.tum_vehicle import TUMVehicle
from driver_dojo.actions.base_actions import BaseActions
import driver_dojo.common.state_variables as state_variables


class SemanticActions(BaseActions):
    def __init__(self):
        super().__init__()
        self._target_speed = None

    def reset(self):
        self._target_speed = state_variables.vehicle.speed
        # If the initial speed is not inside our map, we choose the nearest
        if self._target_speed not in self._speed_map:
            self._target_speed = self._speed_map[
                np.argmin(np.abs(self._speed_map - self._target_speed))
            ]

    def step(self, action):
        # Extracts previous speed_action
        speed_action = np.argwhere(self._speed_map == self._target_speed).flatten()[0]
        road_action = state_variables.vehicle.waypoint_manager.road_option

        # Noop action
        if action == self.space.n - 1:
            pass
        elif state_variables.config["actions"]["disc_hie_cross_prod"]:
            # a_0 -> (s_0, r_0), a_1 -> (s_1, r_0), ... a_m -> (s_m, r_0), a_{m+1} -> (s_0, r_1), ... a_{m+n} (s_m, r_n)
            # where m = num_target_speeds (or 2 if accel action space) and n = num_road_options
            speed_action = int(action / self._num_speed_actions)
            if state_variables.config["actions"]["hie_accel"]:
                speed_action = np.argwhere(
                    self._speed_map == self._target_speed
                ).flatten()[0]
                # Decel
                if speed_action == 0:
                    speed_action = max(0, speed_action - 1)
                else:  # Accel
                    speed_action = min(speed_action + 1, self._num_target_speeds)

            road_action = action % self._num_road_actions
        else:
            # First m action are target_speed action, the action afterwards are road_option action.
            # The one that wasn't picked is taken over from the last time step.
            if action < self._num_speed_actions:
                if state_variables.config["actions"]["hie_accel"]:
                    # Decel
                    if action == 0:
                        speed_action = max(0, speed_action - 1)
                    else:  # Accel
                        speed_action = min(
                            speed_action + 1, self._num_target_speeds - 1
                        )
                else:
                    speed_action = action
            else:
                road_action = action - self._num_speed_actions

        # Set target_speed and road_option
        self._target_speed = self._speed_map[speed_action]

        state_variables.vehicle.waypoint_manager.apply_road_option(road_action)

        next_waypoint = state_variables.vehicle.waypoints[0]
        # Control the vehicle
        if isinstance(state_variables.vehicle, TPSVehicle):
            state_variables.vehicle.control(
                next_waypoint.position, self._target_speed
            )
        elif isinstance(state_variables.vehicle, TUMVehicle):
            pass
            # TODO: PID
            # assert state_variables.config["actions"][
            #     "hie_use_pid"
            # ], "Hierarchical action space with physical vehicle model is only possible when using a PID controller!"
            # accel, steering_velocity = self._pid_controller.run_step(
            #     self._target_speed, self._waypoints[0]
            # )
            # steering_velocity = (
            #     state_variables.vehicle.tum_params.steering.v_max * steering_velocity
            # )
            # accel = state_variables.vehicle.tum_params.longitudinal.a_max * accel
            # state_variables.vehicle.control(steering_velocity, accel)

    @property
    def space(self):
        if self._action_space is not None:
            return self._action_space

        # Define action space
        self._num_target_speeds = state_variables.config["actions"][
            "hie_num_target_speeds"
        ]
        self._speed_map = np.linspace(
            state_variables.vehicle.v_min,
            state_variables.vehicle.v_max,
            num=self._num_target_speeds,
            endpoint=True,
        )
        self._num_speed_actions = (
            self._num_target_speeds
            if not state_variables.config["actions"]["hie_accel"]
            else 2
        )
        self._num_road_actions = len(self.RoadOptions)

        if state_variables.config["actions"]["disc_hie_cross_prod"]:
            self._action_space = Discrete(
                self._num_speed_actions * self._num_road_actions + 1
            )  # +1 for Noop
        else:
            self._action_space = Discrete(
                self._num_speed_actions + self._num_road_actions + 1
            )

        return self._action_space
