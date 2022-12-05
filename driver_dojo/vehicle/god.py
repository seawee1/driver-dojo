from copy import deepcopy
import numpy as np

from driver_dojo.common.traffic_manager import ActorState
from driver_dojo.vehicle import BaseVehicle


class GodVehicle(BaseVehicle):
    """The GodVehicle is a vehicle that is able to perfectly
    track any sequence of waypoints (course). It does so by 'levitating' over the desired course,
    without constrained through an underlying dynamics model.
    """

    def control(self, target_course, target_vel):
        # self._state = self.traffic_manager.get_actor_state(self._veh_id)  # Get most recent ego state

        state = deepcopy(self._state)
        dt = self.config.simulation.dt

        meters_traveled = dt * state.velocity  # Update x,y by shifting the vehicle over the course path
        course_xs, course_ys, course_yaw, _, _ = target_course
        course_res = self.config.vehicle.course_resolution
        course_idx = int(meters_traveled / course_res)
        x_0, y_0, yaw_0 = course_xs[course_idx], course_ys[course_idx], course_yaw[course_idx]
        x_1, y_1, yaw_1 = course_xs[course_idx + 1], course_ys[course_idx + 1], course_yaw[course_idx + 1]
        alpha = (meters_traveled % course_res) / course_res
        x = (1 - alpha) * x_0 + alpha * x_1
        y = (1 - alpha) * y_0 + alpha * y_1
        yaw = (1 - alpha) * yaw_0 + alpha * yaw_1
        state.location[0] = x
        state.location[1] = y
        state.rotation[1] = yaw

        if target_vel - state.velocity > 0.0:  # Update vehicle speed
            accel = self.config.vehicle.accel_max
        elif target_vel - state.velocity < 0.0:
            accel = self.config.vehicle.decel_max
        else:
            accel = 0.0

        if accel >= 0.0:
            state.accel = accel
        else:
            state.decel = accel
        state.velocity += accel * dt
        state.velocity = np.clip(state.velocity, self._veh_config.v_min, self._veh_config.v_max)

        # Avoids oscillations when actually wanting to come to full stop
        if abs(state.velocity) < 1.0 and target_vel == 0.0:  # < 3.6 kilometres per hour
            state.velocity = 0.0

        new_state = ActorState(
            veh_id=state.veh_id,
            location=state.location,
            rotation=state.rotation,
            velocity=state.velocity,
            extent=state.extent,
            sumo_repr=False,
        )
        self.state = new_state

    @property
    def steer(self):
        return 0.0
