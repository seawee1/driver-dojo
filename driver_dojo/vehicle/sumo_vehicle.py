from driver_dojo.vehicle.base_vehicle import BaseVehicle
from driver_dojo.common import runtime_vars


class SUMOVehicle(BaseVehicle):
    def __init__(self, config, egoID, traffic_manager, dt):
        super().__init__(config, egoID, traffic_manager, dt)

    def control(self, action):
        if self.waypoint_manager is not None:
            self.waypoint_manager.step()
        if self.sub_goal_manager is not None:
            self.sub_goal_manager.step()
        if self.traffic_randomizer is not None:
            self.traffic_randomizer.step()

    def reset(self):
        super().reset()

        # Reset attached navigation
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
        return runtime_vars.traffic_manager.ego_state_transformed["position"]

    @property
    def speed(self):
        return runtime_vars.traffic_manager.ego_state_transformed["speed"]

    @property
    def steering_angle(self):
        return 0.0

    @property
    def angle(self):
        return runtime_vars.traffic_manager.ego_state_transformed["angle"]

    @property
    def acceleration(self):
        return runtime_vars.traffic_manager.ego_state_transformed["acceleration"]
