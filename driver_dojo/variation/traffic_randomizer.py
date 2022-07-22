import driver_dojo.common.runtime_vars as runtime_vars
from driver_dojo.common import utils
import numpy as np
import copy
import logging


class TrafficRandomizer:
    def __init__(self):
        self._eb = runtime_vars.config["variation"]["emergency_break"]
        self._eb_interval = runtime_vars.config["variation"][
            "emergency_break_interval"
        ]
        self._eb_break_time = 3.0
        self._eb_stand_still_time = 10.0
        self._ts = runtime_vars.config["variation"]["traffic_speed"]
        self._ts_interval = runtime_vars.config["variation"][
            "traffic_speed_interval"
        ]
        self._ts_range = runtime_vars.config["variation"]["traffic_speed_mult_range"]
        self._ts_duration = self._ts_interval

        self._eb_time_step = None
        self._eb_vehs = None
        self._ts_time_step = None

    def reset(self):
        self._eb_time_step = 0.0
        self._ts_time_step = 0.0
        self._eb_vehs = []

    def step(self):
        if self._eb:
            # Stand still after emergency break
            _eb_vehs = []
            for t, vehID in self._eb_vehs:
                # TODO: What if vehicle already despawned
                if (
                    runtime_vars.time_step
                    * runtime_vars.config["simulation"]["dt"]
                    - t * runtime_vars.config["simulation"]["dt"]
                    >= self._eb_break_time
                ):
                    runtime_vars.traci.vehicle.slowDown(
                        vehID, 0.0, self._eb_stand_still_time
                    )
                else:
                    _eb_vehs.append([t, vehID])
            self._eb_vehs = _eb_vehs

            trafficIDs = copy.deepcopy(runtime_vars.traffic_manager.trafficIDs)
            runtime_vars.np_random_traffic.shuffle(trafficIDs)
            # Emergency break
            if (
                runtime_vars.time_step * runtime_vars.config["simulation"]["dt"]
                - self._eb_time_step * runtime_vars.config["simulation"]["dt"]
                >= self._eb_interval
            ):
                for vehID in trafficIDs:
                    veh_idx = runtime_vars.traffic_manager.trafficIDs.index(vehID)
                    # Check if vehicle in-front of ego and inside cone
                    to_vec = runtime_vars.traffic_manager.traffic_state_transformed(
                        "position"
                    )[veh_idx] - runtime_vars.traffic_manager.ego_state_transformed(
                        "position"
                    )
                    to_angle = utils.wrap_to_pi(utils.vector2d_rad(*to_vec))
                    ego_angle = utils.wrap_to_pi(
                        runtime_vars.traffic_manager.ego_state_transformed("angle")
                    )
                    if abs(ego_angle - to_angle) < np.radians(
                        35
                    ):  # 70 degree cone opening
                        self._eb_vehs.append([runtime_vars.time_step, vehID])
                        runtime_vars.traci.vehicle.slowDown(
                            vehID, 0.0, self._eb_break_time
                        )
                        self._eb_time_step = runtime_vars.time_step
                        break
                    else:
                        continue

        if self._ts:
            # Speed variation
            if (
                runtime_vars.time_step * runtime_vars.config["simulation"]["dt"]
                - self._ts_time_step * runtime_vars.config["simulation"]["dt"]
                >= self._ts_interval
                and len(runtime_vars.traffic_manager.trafficIDs) > 0
            ):
                self._ts_time_step = runtime_vars.time_step
                mult_factor = runtime_vars.np_random_traffic.uniform(
                    low=self._ts_range[0], high=self._ts_range[1]
                )
                vehID = runtime_vars.np_random_traffic.choice(
                    runtime_vars.traffic_manager.trafficIDs
                )
                old_speed = runtime_vars.traci.vehicle.getSpeed(vehID)
                speed = runtime_vars.traci.vehicle.getSpeed(vehID) * mult_factor
                runtime_vars.traci.vehicle.slowDown(vehID, speed, self._ts_duration)
                logging.debug(f"Adapted speed of {vehID} from {old_speed} to {speed}.")
