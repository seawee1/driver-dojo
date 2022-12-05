from copy import deepcopy

import numpy as np
from scipy.integrate import odeint

from driver_dojo.common.traffic_manager import ActorState
from driver_dojo.vehicle import BaseVehicle

from vehiclemodels.init_ks import init_ks
from vehiclemodels.init_st import init_st
from vehiclemodels.init_std import init_std
from vehiclemodels.init_mb import init_mb
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3

from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std
from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb

from driver_dojo.core.types import DynamicsModel, CarModel


class TUMVehicle(BaseVehicle):
    """
    Actions (Cont.):
        0: steering angle velocity of front wheels
        1: longitudinal acceleration
    """
    def __init__(self, config, traffic_manager):
        super().__init__(config, traffic_manager)

        self._car_model_to_params = {
            CarModel.FordEscort: parameters_vehicle1,
            CarModel.BMW320i: parameters_vehicle2,
            CarModel.VWVanagon: parameters_vehicle3,
        }

        self._dyn_model_info = {
            DynamicsModel.KS: (5, init_ks, vehicle_dynamics_ks),
            DynamicsModel.ST: (7, init_st, vehicle_dynamics_st),
            DynamicsModel.STD: (9, init_std, vehicle_dynamics_std),
            DynamicsModel.MB: (29, init_mb, vehicle_dynamics_mb),
        }

        self._state_internal = None
        self._vehicle_config = self.config.vehicle

        self._dynamics_params = None  # We can only query them at first reset
        self._state_dim, self._init_func, self._dynamic_func = self._dyn_model_info[self._vehicle_config.dynamics_model]
        self._init_dynamics_params()

    def control(self, steer_vel, accel):
        # Predominantly used for better vehicle control via `manual_control.py`.
        # See: https://en.wikipedia.org/wiki/Caster_angle
        # We apply a steering velocity going into the neutral steering direction whenever steering_velocity input is 0.
        # The steering velocity applied is speed dependent, i.e.
        # f(speed) ~ min(speed/(2*13.889), 1.0) where we apply max steering velocity at >= 100 km/h
        # (27.778 m/s) and a linearly decreasing factor at < 100 km/h.
        if self._vehicle_config["caster_effect"] and steer_vel == 0.0:
            factor = np.abs(min(self._velocity / (2 * 13.889) + 0.2, 1.0))
            if abs(self.steer) <= np.pi/16:
                self.steer = 0.0
            else:
                steer_vel = (
                        -np.sign(self.steer) * np.sign(self._velocity) * factor
                )

        def func(state, t, u, p):  # Update vehicle based on state and inputs
            f = self._dynamic_func(state, u, p)
            return f

        self._state_internal = odeint(
            func,
            self._state_internal,
            t=[0, self.config.simulation.dt],
            args=(np.array([steer_vel, accel]), self._dynamics_params),
        )[-1]

        new_state = ActorState(
            veh_id=self.state.veh_id,
            location=np.array([self._location[0], self._location[1], 0.0]),
            rotation=np.array([self._state.rotation[0], self._rotation, self._state.rotation[2]]),
            velocity=self._velocity,
            extent=self._state.extent,
            sumo_repr=False,
        )
        if accel >= 0.0:
            new_state.accel = accel
        else:
            new_state.decel = accel

        self.state = new_state

    def _init_dynamics_params(self):
        """Overrides the internal TUM dynamics model parameters with configurations specified inside `Config.vehicle`."""
        self._dynamics_params = self._car_model_to_params[self._vehicle_config.car_model]()
        self._dynamics_params.w = self._vehicle_config.width
        self._dynamics_params.l = self._vehicle_config.length
        self._dynamics_params.longitudinal.v_max = self._vehicle_config.v_max
        self._dynamics_params.longitudinal.v_min = self._vehicle_config.v_min
        self._dynamics_params.longitudinal.a_max = self._vehicle_config.accel_max

    def reset(self):
        super().reset()
        # In general, i.e. for all dynamics models, we have internal state vector with first entries:
        # [x, y, steer, v, theta (angle), ...]
        self._state_internal = np.zeros(self._state_dim)
        self._location = self._state.location[:2]
        self.steer = 0.0
        self._velocity = self._state.velocity
        self._rotation = self._state.rotation[1]
        self._state_internal = self._init_func(self._state_internal)

    @property
    def state_internal(self):
        return self._state_internal

    @property
    def dynamics_params(self):
        return self._dynamics_params

    @property
    def _x(self):
        return self._state_internal[0]

    @_x.setter
    def _x(self, x):
        self._state_internal[0] = x

    @property
    def _y(self):
        return self._state_internal[1]

    @_y.setter
    def _y(self, x):
        self._state_internal[1] = x

    @property
    def _location(self):
        return self._state_internal[:2]

    @_location.setter
    def _location(self, x):
        self._state_internal[:2] = x

    @property
    def steer(self):
        return self._state_internal[2]

    @steer.setter
    def steer(self, x):
        self._state_internal[2] = x

    @property
    def _velocity(self):
        return self._state_internal[3]

    @_velocity.setter
    def _velocity(self, x):
        self._state_internal[3] = x

    @property
    def _rotation(self):
        return self._state_internal[4]

    @_rotation.setter
    def _rotation(self, x):
        self._state_internal[4] = x
