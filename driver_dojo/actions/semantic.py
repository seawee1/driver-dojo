import numpy as np
from gym.spaces import Discrete

from driver_dojo.actions import BaseActions
from driver_dojo.vehicle import GodVehicle
from driver_dojo.vehicle import TUMVehicle


class SemanticActions(BaseActions):
    def __init__(self, config, vehicle):
        super().__init__(config, vehicle)
        self._num_road_actions = None
        self._num_speed_actions = None
        self._num_target_speeds = None
        self._speed_map = None
        self._target_speed = None

    def reset(self):
        self._target_speed = self.vehicle.velocity
        # If the initial speed is not inside our map, we choose the nearest
        if self._speed_map is None:
            self._speed_map = np.linspace(
                # world.world.vehicle.velocity_min,  # TODO: Fix bug where TrafficManager overwrites these with None
                0.0,
                self.vehicle.velocity_max,
                num=self._num_target_speeds,
                endpoint=True,
            )

        if self._target_speed not in self._speed_map:
            self._target_speed = self._speed_map[
                np.argmin(np.abs(self._speed_map - self._target_speed))
            ]

    def step(self, action):
        # Extracts previous speed_action
        speed_action = np.argwhere(self._speed_map == self._target_speed).flatten()[0]
        road_action = self.vehicle.waypoint_attachment.cur_road_option

        # Noop action
        if action == self.action_space.n - 1:
            pass
        elif self.config["actions"]["disc_hie_cross_prod"]:
            # a_0 -> (s_0, r_0), a_1 -> (s_1, r_0), ... a_m -> (s_m, r_0), a_{m+1} -> (s_0, r_1), ... a_{m+n} (s_m, r_n)
            # where m = num_target_speeds (or 2 if accel action space) and n = num_road_options
            speed_action = int(action / self._num_speed_actions)
            if self.config["actions"]["hie_accel"]:
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
                if self.config["actions"]["hie_accel"]:
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

        self.vehicle.waypoint_attachment.apply_road_option(road_action)

        # Control the vehicle
        course = self.vehicle.waypoint_attachment.cubic_spline_course(self.vehicle.state.location[:2], res=self.config.vehicle.course_resolution)
        if isinstance(self.vehicle, GodVehicle):
            self.vehicle.control(course, self._target_speed)
        elif isinstance(self.vehicle, TUMVehicle):  # Use control method for course tracking
            from PathTracking.stanley_controller import stanley_controller
            stanley_controller.k = 0.9  # TODO: Lane-change and tight curves are bit problematic to track right now. Maybe Stanley isn't the right method.
            stanley_controller.Kp = 1.0
            stanley_controller.dt = self.config.simulation.dt
            steering_v_max_frac = 0.8

            state = stanley_controller.State(  # Prepare state
                x=self.vehicle.location[0],
                y=self.vehicle.location[1],
                yaw=self.vehicle.rotation[1] + self.vehicle.steer,
                v=self.vehicle.velocity
            )
            cx, cy, cyaw, ck, s = course
            target_idx, _ = stanley_controller.calc_target_index(state, cx, cy)
            accel = stanley_controller.pid_control(self._target_speed, state.v)
            steer, _ = stanley_controller.stanley_control(state, cx, cy, cyaw, target_idx)
            steer_vel = steer/self.config.simulation.dt
            steer_vel_min, steer_vel_max = steering_v_max_frac * self.vehicle.dynamics_params.steering.v_min, steering_v_max_frac * self.vehicle.dynamics_params.steering.v_max
            steer_vel = np.clip(steer_vel, steer_vel_min, steer_vel_max)
            self.vehicle.control(steer_vel, accel)

    @property
    def action_space(self):
        if self._action_space is not None:
            return self._action_space

        # Define action space
        self._num_target_speeds = self.config["actions"][
            "hie_num_target_speeds"
        ]
        self._num_speed_actions = (
            self._num_target_speeds
            if not self.config["actions"]["hie_accel"]
            else 2
        )
        self._num_road_actions = len(self._road_options)

        if self.config["actions"]["disc_hie_cross_prod"]:
            self._action_space = Discrete(
                self._num_speed_actions * self._num_road_actions + 1
            )  # +1 for Noop
        else:
            self._action_space = Discrete(
                self._num_speed_actions + self._num_road_actions + 1
            )

        return self._action_space

