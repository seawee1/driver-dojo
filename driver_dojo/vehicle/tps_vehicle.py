import numpy as np

from driver_dojo.common import utils
from driver_dojo.vehicle.base_vehicle import BaseVehicle
import driver_dojo.common.runtime_vars as runtime_vars


class TPSVehicle(BaseVehicle):
    """The TargetSpeedPositionVehicle (TPSVehicle) can be seen as a vehicle that is able to perfectly track any sequence of waypoints.
    """

    def __init__(self, config, egoID, traffic_manager, dt):
        super().__init__(config, egoID, traffic_manager, dt)

        self._position, self._speed, self._angle = None, None, None
        self._acceleration = None

    def control(self, target_position, target_speed):
        # Get unit direction vector towards target_waypoint
        target_position = (
            np.array(target_position)
            if not isinstance(target_position, np.ndarray)
            else target_position
        )
        direction = target_position - self.position
        norm = np.linalg.norm(direction)
        norm = 1.0 if norm == 0.0 else norm
        direction = direction / norm

        # Adjust yaw angle toward next waypoint
        angle = utils.vector2d_rad(*direction)
        match_angle = utils.rad_difference(self.angle, angle)
        match_angle = utils.wrap_to_pi(match_angle)
        # Rotate with np.pi/3.0 rad/s (60 degrees per second) at 14.0 m/s (~50 km/h)
        # Linearly scale with speed
        yaw_step = np.pi / 3.0 * self.dt * self.speed / 14.0
        # If the angle between ego and next waypoint is bigger than max_angle, increase rotational speed linearly by factor match_angle - max_angle
        # This allows for more realistic behavior when driving tight curves with high speed
        max_angle = 0.262  # 10 degrees
        addition_scaling = max(0.0, abs(match_angle) - max_angle) + 1.0

        # Calculate change applied to current yaw
        yaw_change = min(abs(match_angle), yaw_step)
        yaw_change *= addition_scaling
        yaw_change *= np.sign(match_angle)
        # Don't adjust when standing still
        if self.speed == 0.0:
            yaw_change = 0.0
        self.angle = self.angle + yaw_change

        # Update vehicle speed
        # Either accel or decel
        if target_speed - self.speed > 0.0:
            self._acceleration = self.accel
        elif target_speed - self.speed < 0.0:
            self._acceleration = self.decel
        self.speed += self._acceleration * self.dt
        # Ensure speed inside min-max speed interval
        self.speed = np.clip(self.speed, self.v_min, self.v_max)
        # Avoids oscillations when actually wanting to come to full stop
        if abs(self.speed) < 1.0 and target_speed == 0.0:  # 3.6 kilometres per hour
            self.speed = 0.0

        # Update position
        self.position = self.position + (direction * self.speed * self.dt)

        # Broadcast to TrafficManager
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

        self._position = runtime_vars.traffic_manager.ego_state_transformed(
            "position"
        )
        self._angle = runtime_vars.traffic_manager.ego_state_transformed("angle")
        self._speed = runtime_vars.traffic_manager.ego_state_transformed("speed")
        self._acceleration = runtime_vars.traffic_manager.ego_state_transformed(
            "acceleration"
        )

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
    def position(self):
        return self._position

    @property
    def speed(self):
        return self._speed

    @property
    def steering_angle(self):
        return 0.0

    @property
    def acceleration(self):
        return self._acceleration

    @property
    def angle(self):
        return self._angle

    @position.setter
    def position(self, position):
        self._set_position(position)
        self._position = (
            np.array(position) if not isinstance(position, np.ndarray) else position
        )

    @speed.setter
    def speed(self, speed):
        self._set_speed(speed)
        self._speed = speed

    @steering_angle.setter
    def steering_angle(self, steering_angle):
        pass

    @angle.setter
    def angle(self, angle):
        self._set_angle(angle)
        self._angle = angle
