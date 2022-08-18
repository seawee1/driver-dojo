import numpy as np
import sumolib
from traci.constants import CMD_GET_VEHICLE_VARIABLE
from traci.constants import (
    VAR_POSITION,
    VAR_ANGLE,
    VAR_SPEED,
    VAR_LENGTH,
    VAR_WIDTH,
    VAR_ACCELERATION,
    VAR_SIGNALS,
)
from traci.constants import VAR_ROAD_ID, VAR_LANE_ID, VAR_LANE_INDEX
import logging

import driver_dojo.common.runtime_vars as runtime_vars
from driver_dojo.common import utils


class TrafficManager:
    def __init__(self):
        self.simulation_config = runtime_vars.config["simulation"]
        self.rand_config = runtime_vars.config["variation"]
        self.radius = self.simulation_config["traffic_manager_radius"]
        self.varIDs = [
            VAR_POSITION,
            VAR_ANGLE,
            VAR_SPEED,
            VAR_LENGTH,
            VAR_WIDTH,
            VAR_ROAD_ID,
            VAR_LANE_ID,
            VAR_LANE_INDEX,
            VAR_ACCELERATION,
            VAR_SIGNALS,
        ]
        self.index = {
            "position": [0, 1],
            "angle": 2,
            "speed": 3,
            "acceleration": 4,
            "length": 5,
            "width": 6,
            "signals": 7,
            "roadID": 0,  # split into ..._state and ..._state_str
            "laneID": 1,
            "lane_index": 2,
        }

        self._ego_state = None
        self._ego_state_transformed = None
        self._ego_state_str = None
        self._traffic_state = None
        self._traffic_state_transformed = None
        self._traffic_state_transformed_relative = None
        self._traffic_state_str = None
        self._trafficIDs = None
        self._dist_to_ego = None

    def _reset_data_structures(self):
        self._ego_state = None
        self._ego_state_transformed = None
        self._ego_state_str = None
        self._traffic_state = None
        self._traffic_state_transformed = None
        self._traffic_state_transformed_relative = None
        self._traffic_state_str = None
        self._trafficIDs = None
        self._dist_to_ego = None

    def reset(self):
        # runtime_vars.net = sumolib.net.readNet(self.simulation_config['net_path'], withInternal=True)
        self._reset_data_structures()

        # Note: other subscriptions filters might be interesting!
        # addSubscriptionFilterCFManeuver: to ego leader and follower
        # addSubscriptionFilterDownstreamDistance: based on downstream network dist
        # addSubscriptionFilterFieldOfVision
        # addSubscriptionFilterLCManeuver
        # addSubscriptionFilterLanes(lanes, ...): more general case of addSubscriptionFilterLCManeuver
        # addSubscriptionFilterLateralDistance
        # addSubscriptionFilterLeadFollow: to neighbor and ego-lane leader and follower of the ego.
        # addSubscriptionFilterNoOpposite: omits vehicle on other edges
        # addSubscriptionFilterTurn: foes on upcoming junction
        runtime_vars.traci.vehicle.subscribeContext(
            self.simulation_config["egoID"],
            CMD_GET_VEHICLE_VARIABLE,
            self.radius,
            varIDs=self.varIDs,
        )
        if runtime_vars.config["simulation"]["filter_traffic"]:
            # SUMO 1.11 release notes: https://www.eclipse.org/lists/sumo-announce/msg00049.html
            # "addSubscriptionFilterTurn can now be combined (additively) with addSubscriptionFilterLateralDistance and with addSubscriptionFilterLanes."
            # -> Traffic filtering can also be implemented using multiple subscription filters, but we found results to be a little unpredictable.
            runtime_vars.traci.vehicle.addSubscriptionFilterTurn(50.0, 20.0)

    def step(self):
        self._reset_data_structures()

        if self.rand_config["traffic_vTypes"]:
            departedIDs = runtime_vars.traci.simulation.getDepartedIDList()
            for departedID in departedIDs:
                if departedID != self.simulation_config["egoID"]:
                    runtime_vars.traci.vehicle.setType(departedID, "vehDist")

        context = runtime_vars.traci.vehicle.getContextSubscriptionResults(
            self.simulation_config["egoID"]
        )

        if runtime_vars.config["simulation"]["filter_traffic"]:
            # TODO: We might loose some performance because of all the TraCI calls. Maybe there's a better way to do this.
            def manually_add_subscription_result(vehID):
                if vehID == "":
                    return
                context[vehID] = {
                    VAR_POSITION: list(
                        runtime_vars.traci.vehicle.getPosition(vehID)
                    ),
                    VAR_ANGLE: runtime_vars.traci.vehicle.getAngle(vehID),
                    VAR_SPEED: runtime_vars.traci.vehicle.getSpeed(vehID),
                    VAR_LENGTH: runtime_vars.traci.vehicle.getLength(vehID),
                    VAR_WIDTH: runtime_vars.traci.vehicle.getWidth(vehID),
                    VAR_ROAD_ID: runtime_vars.traci.vehicle.getRoadID(vehID),
                    VAR_LANE_ID: runtime_vars.traci.vehicle.getLaneID(vehID),
                    VAR_LANE_INDEX: runtime_vars.traci.vehicle.getLaneIndex(vehID),
                    VAR_ACCELERATION: runtime_vars.traci.vehicle.getAcceleration(
                        vehID
                    ),
                    VAR_SIGNALS: runtime_vars.traci.vehicle.getSignals(vehID),
                }

            # We have to manually add the ego via TraCI requests
            egoID = runtime_vars.config["simulation"]["egoID"]
            manually_add_subscription_result(egoID)

            # We then add followers and leaders to traffic of interest
            # print(follower, leader, left_followers, right_followers, left_leaders, right_leaders)
            # -> result: ('', -1.0) None () () () ()
            vehIDs = []
            follower = runtime_vars.traci.vehicle.getFollower(egoID)
            if follower[0] != "":
                vehIDs.append(follower[0])
            leader = runtime_vars.traci.vehicle.getLeader(egoID)
            if leader is not None:
                vehIDs.append(leader[0])
            [
                vehIDs.append(x[0])
                for x in runtime_vars.traci.vehicle.getLeftFollowers(egoID)
            ]
            [
                vehIDs.append(x[0])
                for x in runtime_vars.traci.vehicle.getRightFollowers(egoID)
            ]
            [
                vehIDs.append(x[0])
                for x in runtime_vars.traci.vehicle.getLeftLeaders(egoID)
            ]
            [
                vehIDs.append(x[0])
                for x in runtime_vars.traci.vehicle.getRightLeaders(egoID)
            ]
            for vehID in vehIDs:
                manually_add_subscription_result(vehID)

        num_traffic = (
            len(context.keys()) - 1
            if self.simulation_config["egoID"] in context.keys()
            else len(context.keys())
        )
        self._traffic_state = np.zeros((num_traffic, 8))
        self._traffic_state_str = dict()
        self._trafficIDs = []

        i = 0
        for k, v in context.items():
            # See: self.var_name_to_index
            data = np.array(
                [
                    v[VAR_POSITION][0],
                    v[VAR_POSITION][1],
                    v[VAR_ANGLE],
                    v[VAR_SPEED],
                    v[VAR_ACCELERATION],
                    v[VAR_LENGTH],
                    v[VAR_WIDTH],
                    v[VAR_SIGNALS],
                ]
            )
            data_other = [v[VAR_ROAD_ID], v[VAR_LANE_ID], v[VAR_LANE_INDEX]]

            if k == self.simulation_config["egoID"]:
                self._ego_state = data
                self._ego_state_str = data_other
            else:
                self._traffic_state[i] = data
                self._traffic_state_str[k] = data_other
                self._trafficIDs.append(k)
                i += 1

            if runtime_vars.config["simulation"]["junction_ignore_ego"]:
                runtime_vars.traci.vehicle.setParameter(
                    k,
                    "junctionModel.ignoreIDs",
                    runtime_vars.config["simulation"]["egoID"],
                )

    def modify_vehicle_state(
        self,
        id,
        xy=None,
        pos=None,
        speed=None,
        yaw=None,
        laneID=None,
        lane_index=None,
        edgeID=None,
        override_signals=False,
    ):
        # logging.info(f"TrafficManager: modify_vehicle_state (id={id}, xy={xy}, pos={pos}, speed={speed}, yaw={yaw}, laneID={laneID}, lane_index={lane_index}, edgeID={edgeID}, override_signals={override_signals})")
        # Read current state
        if id == self.simulation_config["egoID"]:
            state = self.ego_state_transformed()
        else:
            if id not in self._trafficIDs:
                raise ValueError(
                    "Tried to change vehicle state of vehicle that lies outside the TrafficManager radius!"
                )
            index = self._trafficIDs.index(id)
            state = self.traffic_state_transformed()[index]

        # Set position along lane
        if pos is not None and laneID is not None:
            x, y = sumolib.geomhelper.positionAtShapeOffset(
                runtime_vars.net.getLane(laneID).getShape(), pos
            )
            xy = np.array([x, y])

        # Set position rotation
        if xy is not None or yaw is not None:
            length = state[self.index["length"]]
            if xy is not None:
                x, y = xy[0], xy[1]
            else:
                x, y = state[self.index["position"]]

            yaw = state[self.index["angle"]] if yaw is None else yaw
            state[self.index["position"]] = np.array([x, y])
            state[self.index["angle"]] = yaw

            x, y, yaw = utils.transform_to_sumo(x, y, yaw, length / 2)
            if edgeID is None:
                edgeID = ""
            if lane_index is None:
                lane_index = -1

            # From: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#move_to_xy_0xb4=
            # bit0 (keepRoute = 1 when only this bit is set)
            #     1:  The vehicle is mapped to the closest edge within it's existing route. If no suitable position is found within 100m mapping fails with an error.
            #     0:  The vehicle is mapped to the closest edge within the network.
            #         If that edge does not belong to the original route, the current route is replaced by a new route which consists  of that edge only.
            #         If no suitable position is found within 100m mapping fails with an error. When using the sublane model the best lateral position that is fully within the lane will be used.
            #         Otherwise, the vehicle will drive in the center of the closest lane.
            # bit1 (keepRoute = 2 when only this bit is set)
            #     1:  The vehicle is mapped to the exact position in the network (including the exact lateral position).
            #         If that position lies outside the road network, the vehicle stops moving on it's own accord until it is placed back into the network with another TraCI command.
            #         (if keeproute = 3, the position must still be within 100m of the vehicle route)
            #     0:  The vehicle is always on a road
            # bit2 (keepRoute = 4 when only this bit is set)
            #     1:  lane permissions are ignored when mapping
            #     0:  The vehicle is mapped only to lanes that allow it's vehicle class
            runtime_vars.traci.vehicle.moveToXY(
                id, edgeID, lane_index, x, y, angle=yaw, keepRoute=1, matchThreshold=200.0,
            )

        # Speed
        if speed is not None:
            runtime_vars.traci.vehicle.setSpeed(id, speed=speed)
            state[self.index["speed"]] = speed

        # Override signaling
        if override_signals:
            runtime_vars.traci.vehicle.setSignals(id, 0)
            state[self.index["signals"]] = 0

        # Write-back new state
        if id == self.simulation_config["egoID"]:
            self._ego_state_transformed = state
            x, y, angle = utils.transform_to_sumo(
                state[self.index["position"][0]],
                state[self.index["position"][1]],
                state[self.index["angle"]],
                state[self.index["length"]] / 2.0,
            )
            self._ego_state[self.index["position"][0]] = x
            self._ego_state[self.index["position"][0]] = y
            self._ego_state[self.index["angle"]] = angle
        else:
            self._traffic_state_transformed[index] = state
            x, y, angle = utils.transform_to_sumo(
                state[self.index["position"][0]],
                state[self.index["position"][1]],
                state[self.index["angle"]],
                state[self.index["length"]] / 2.0,
            )
            self._traffic_state[self.index["position"][0]] = x
            self._traffic_state[self.index["position"][0]] = y
            self._traffic_state[self.index["angle"]] = angle

    def traffic_state(self, var_name=None):
        return (
            self._traffic_state[self.index[var_name]]
            if var_name
            else self._traffic_state
        )

    def ego_state(self, var_name=None):
        return self._ego_state[self.index[var_name]] if var_name else self._ego_state

    def traffic_state_str(self, var_name=None):
        return (
            self._traffic_state_str[self.index[var_name]]
            if var_name
            else self._traffic_state_str
        )

    def ego_state_str(self, var_name=None):
        return (
            self._ego_state_str[self.index[var_name]]
            if var_name
            else self._ego_state_str
        )

    def ego_state_transformed(self, var_name=None):
        if self._ego_state_transformed is not None:
            return (
                self._ego_state_transformed[self.index[var_name]]
                if var_name
                else self._ego_state_transformed
            )

        x = np.copy(self.ego_state())
        if x is None:
            assert x is not None

        (
            x[self.index["position"][0]],
            x[self.index["position"][1]],
            x[self.index["angle"]],
        ) = utils.transform_from_sumo(
            x[self.index["position"][0]],
            x[self.index["position"][1]],
            x[self.index["angle"]],
            x[self.index["length"]] / 2.0,
        )

        x[self.index["angle"]] = utils.wrap_to_pi(x[self.index["angle"]])
        self._ego_state_transformed = x
        return (
            self._ego_state_transformed[self.index[var_name]]
            if var_name
            else self._ego_state_transformed
        )

    def traffic_state_transformed(self, var_name=None):
        if self._traffic_state_transformed is not None:
            return (
                self._traffic_state_transformed[:, self.index[var_name]]
                if var_name
                else self._traffic_state_transformed
            )
        elif self._traffic_state.shape[0] == 0:
            return None

        x = np.copy(self._traffic_state)
        (
            x[:, self.index["position"][0]],
            x[:, self.index["position"][1]],
            x[:, self.index["angle"]],
        ) = utils.transform_from_sumo(
            x[:, self.index["position"][0]],
            x[:, self.index["position"][1]],
            x[:, self.index["angle"]],
            x[:, self.index["length"]] / 2.0,
        )
        x[:, self.index["angle"]] = utils.wrap_to_pi(x[:, self.index["angle"]])

        self._traffic_state_transformed = x
        return (
            self._traffic_state_transformed[:, self.index[var_name]]
            if var_name
            else self._traffic_state_transformed
        )

    def traffic_state_transformed_relative(self, var_name=None):
        if self._traffic_state_transformed_relative is not None:
            return (
                self._traffic_state_transformed_relative[:, self.index[var_name]]
                if var_name
                else self._traffic_state_transformed_relative
            )
        elif self._traffic_state.shape[0] == 0:
            return None

        x = np.copy(self.traffic_state_transformed())
        y = self.ego_state_transformed()
        x[:, self.index["position"]] = (
            x[:, self.index["position"]] - y[self.index["position"]]
        )
        x[:, self.index["angle"]] = utils.rad_difference(
            x[:, self.index["angle"]], y[self.index["angle"]]
        )
        x[:, self.index["angle"]] = utils.wrap_to_pi(x[:, self.index["angle"]])

        theta = -y[self.index["angle"]] + np.radians(90)
        rot_mat = np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )
        x_rot = np.dot(rot_mat, x[:, self.index["position"]].T).T
        x[:, self.index["position"]] = x_rot

        x[:, self.index["speed"]] = x[:, self.index["speed"]] - y[self.index["speed"]]
        x[:, self.index["acceleration"]] = (
            x[:, self.index["acceleration"]] - y[self.index["acceleration"]]
        )

        self._traffic_state_transformed_relative = x
        return (
            self._traffic_state_transformed_relative[:, self.index[var_name]]
            if var_name
            else self._traffic_state_transformed_relative
        )

    @property
    def distance_to_ego(self):
        if self._dist_to_ego is not None:
            return self._dist_to_ego
        elif self._traffic_state.shape[0] == 0:
            return None

        x = self.traffic_state_transformed_relative()[:, self.index["position"]]
        self._dist_to_ego = np.linalg.norm(x, axis=1)
        return self._dist_to_ego

    @property
    def ego_params(self):
        return dict(
            accel=runtime_vars.traci.vehicle.getAccel(
                self.simulation_config["egoID"]
            ),
            decel=runtime_vars.traci.vehicle.getDecel(
                self.simulation_config["egoID"]
            ),
            v_min=0.0,
            v_max=runtime_vars.traci.vehicle.getMaxSpeed(
                self.simulation_config["egoID"]
            ),
            length=runtime_vars.traci.vehicle.getLength(
                self.simulation_config["egoID"]
            ),
            width=runtime_vars.traci.vehicle.getWidth(
                self.simulation_config["egoID"]
            ),
        )

    @ego_params.setter
    def ego_params(self, params):
        runtime_vars.traci.vehicle.setAccel(
            self.simulation_config["egoID"], params["accel"]
        )
        runtime_vars.traci.vehicle.setDecel(
            self.simulation_config["egoID"], -params["decel"]
        )
        runtime_vars.traci.vehicle.setMaxSpeed(
            self.simulation_config["egoID"], params["v_max"]
        )
        runtime_vars.traci.vehicle.setLength(
            self.simulation_config["egoID"], params["length"]
        )
        runtime_vars.traci.vehicle.setWidth(
            self.simulation_config["egoID"], params["width"]
        )

    @property
    def trafficIDs(self):
        return self._trafficIDs
