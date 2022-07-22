import numpy as np
from scipy.integrate import odeint

from driver_dojo.vehicle.base_vehicle import BaseVehicle

from driver_dojo.vehicle.vehiclemodels.init_ks import init_ks
from driver_dojo.vehicle.vehiclemodels.init_st import init_st
from driver_dojo.vehicle.vehiclemodels.init_std import init_std
from driver_dojo.vehicle.vehiclemodels.init_mb import init_mb

from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std
from driver_dojo.vehicle.vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb

import driver_dojo.common.runtime_vars as runtime_vars


class TUMVehicle(BaseVehicle):
    """
    Actions (Cont.):
        0: steering angle velocity of front wheels
        1: longitudinal acceleration
    """

    def __init__(self, config, egoID, traffic_manager, dt):
        super().__init__(config, egoID, traffic_manager, dt)

        from driver_dojo.core.types import DynamicsModel

        vehicle_map = {
            DynamicsModel.KS: (5, init_ks, vehicle_dynamics_ks),
            DynamicsModel.ST: (7, init_st, vehicle_dynamics_st),
            DynamicsModel.STD: (9, init_std, vehicle_dynamics_std),
            DynamicsModel.MB: (29, init_mb, vehicle_dynamics_mb),
        }

        self._state = None
        self._acceleration = None
        self.state_dim, self.init_func, self.dynamic_func = vehicle_map[
            self.vehicle_config["dynamics_model"]
        ]

    def control(self, steering_velocity, acceleration):
        # See: https://en.wikipedia.org/wiki/Caster_angle
        # Here, we apply a steering velocity going into the
        # neutral steering direction whenever steering_velocity input is 0. The steering velocity applied is speed
        # dependent, i.e. f(speed) ~ min(speed/(2*13.889), 1.0) where we apply max steering velocity at >= 100 km/h
        # (27.778 m/s) and a linearly decreasing factor at < 100 km/h.
        if self.vehicle_config["caster_effect"] and steering_velocity == 0.0:
            factor = np.abs(min(self.speed / (2 * 13.889) + 0.2, 1.0))
            steering_velocity = (
                -np.sign(self.steering_angle) * np.sign(self.speed) * factor
            )

        # Do one update step of vehicle state
        def func(state, t, u, p):
            f = self.dynamic_func(state, u, p)
            return f

        self._state = odeint(
            func,
            self._state,
            t=[0, self.dt],
            args=(np.array([steering_velocity, acceleration]), self.tum_params),
        )[-1]
        self._acceleration = acceleration

        # Broadcasts to TrafficManager
        runtime_vars.traffic_manager.modify_vehicle_state(
            self.egoID,
            xy=self.position,
            speed=self.speed,
            yaw=self.angle,
            override_signals=True,
        )

        if self.waypoint_manager is not None:
            self.waypoint_manager.step()
        if self.sub_goal_manager is not None:
            self.sub_goal_manager.step()
        if self.traffic_randomizer is not None:
            self.traffic_randomizer.step()

    def reset(self):
        super().reset()

        # In general, we have state vector with first entries
        # 0: x
        # 1: y
        # 2: steering angle of front wheels
        # 3: velocity in x-direction
        # 4: yaw angle
        self._state = np.zeros(self.state_dim)
        self.steering_angle = 0.0
        self.position = runtime_vars.traffic_manager.ego_state_transformed(
            "position"
        )
        self.angle = runtime_vars.traffic_manager.ego_state_transformed("angle")
        self.speed = runtime_vars.traffic_manager.ego_state_transformed("speed")
        self._acceleration = runtime_vars.traffic_manager.ego_state_transformed(
            "acceleration"
        )
        self._state = self.init_func(self._state)

        # Reset attached modules
        if self.waypoint_manager is not None:
            self.waypoint_manager.reset()
            self.waypoint_manager.step()
        if self.sub_goal_manager is not None:
            self.sub_goal_manager.reset()
            self.waypoint_manager.step()
        if self.traffic_randomizer is not None:
            self.traffic_randomizer.reset()
            self.waypoint_manager.step()

    @property
    def state(self):
        return self._state

    @property
    def position(self):
        return self._state[0:2]

    @property
    def speed(self):
        return self._state[3]

    @property
    def acceleration(self):
        return self._acceleration

    @property
    def steering_angle(self):
        return self._state[2]

    @property
    def angle(self):
        return self._state[4]

    @position.setter
    def position(self, position):
        self._set_position(position)
        self._state[0] = position[0]
        self._state[1] = position[1]

    @speed.setter
    def speed(self, speed):
        self._set_speed(speed)
        self._state[3] = speed

    @steering_angle.setter
    def steering_angle(self, steering_angle):
        self._state[2] = steering_angle

    @angle.setter
    def angle(self, angle):
        self._set_angle(angle)
        self._state[4] = angle
