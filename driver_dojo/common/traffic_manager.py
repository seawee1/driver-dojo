import numpy as np
from dataclasses import dataclass, fields
from typing import List, Optional, Union, Tuple
from copy import deepcopy
import traci

from driver_dojo.common.utils import wrap_to_pi


def del_list_indices(lst: List, index: List[int]):
    index.sort()
    for i, k in enumerate(index):
        del lst[k - i]
    return lst


# TODO: SimulationState, i.e. https://sumo.dlr.de/docs/TraCI/Simulation_Value_Retrieval.html
@dataclass
class ActorState:
    veh_id: str = None
    veh_type: str = None
    veh_class: str = None
    # color: np.ndarray = None
    length: float = None
    width: float = None
    height: float = None
    rotation: np.ndarray = None
    location: np.ndarray = None
    lane_position: float = None  # TODO
    lane_position_lat: float = None  # TODO
    velocity: float = None
    velocity_lat: float = None
    velocity_min: float = None
    velocity_max: float = None
    velocity_allowed: float = None  # TODO
    accel: float = None
    decel: float = None
    accel_max: float = None
    decel_max: float = None
    signals: int = None
    extent: np.ndarray = None
    edge_id: str = None
    lane_id: str = None
    lane_index: int = None
    route: np.ndarray = None
    route_id: str = None  # TODO
    route_index: int = None  # TODO
    distance_driven: float = None  # TODO
    sumo_repr: bool = None
    shape: np.ndarray = None  # TODO

    # We could add a lot more of extra state infos, see SUMO docs
    # https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html:
    #      - distance request
    #      - neighboring vehicles
    #      - leader
    #      - next_tls

    def to_universal(self):
        if not self.sumo_repr:
            return self
        s = deepcopy(self)
        s.rotation[1] = np.radians(-s.rotation[1] + 90.0)
        forward_vector = np.array([np.cos(s.rotation[1]), np.sin(s.rotation[1]), np.sin(s.rotation[0])])
        s.location -= forward_vector * s.extent[0]
        s.sumo_repr = False
        return s

    def to_sumo(self):
        if self.sumo_repr:
            return self

        s = deepcopy(self)
        forward_vector = np.array([np.cos(s.rotation[1]), np.sin(s.rotation[1]), np.sin(s.rotation[0])])
        s.location += forward_vector * s.extent[0]
        s.rotation[1] = -np.degrees(s.rotation[1]) + 90.0
        s.sumo_repr = True
        return s

    def relative_to(self, context_state):
        """This method makes location, rotation, velocity, accel and decel relative to a context actor."""
        s = deepcopy(self)
        context_state = deepcopy(context_state)

        s.location -= context_state.location  # TODO: The error is already before this point
        s.rotation = wrap_to_pi(-context_state.rotation + s.rotation)
        rot = -context_state.rotation[1] + np.radians(90)
        rot_mat = np.array(
            [[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]]
        )
        s.location = np.dot(rot_mat, s.location[:2].T).T  # We assume roads to be completely flat for now
        for attr_name in [
            'length', 'width', 'height',
            'velocity', 'velocity_lat',
            'velocity_max', 'accel', 'decel',
            'accel_max', 'decel_max', 'extent'
        ]:
            setattr(s, attr_name, getattr(s, attr_name) - getattr(context_state, attr_name))

        if self.sumo_repr:
            s = s.to_sumo()
        return s

    def distance_to(self, context_state):
        return np.linalg.norm(self.location - context_state.location)

    def __setattr__(self, key, value):
        # Automatic casting
        if isinstance(value, List):
            value = np.array(value)
        super().__setattr__(key, value)


@dataclass(init=False)
class TrafficState:
    veh_id: np.ndarray  # TODO: numpy type hinting https://stackoverflow.com/questions/70714087/how-to-typehint-numpy-array-of-floats-and-or-integers
    veh_type: np.ndarray
    veh_class: np.ndarray
    # color: np.ndarray  # Not supported by libsumo
    length: np.ndarray
    width: np.ndarray
    height: np.ndarray
    location: np.ndarray
    rotation: np.ndarray
    velocity: np.ndarray
    velocity_lat: np.ndarray
    velocity_min: np.ndarray
    velocity_max: np.ndarray
    accel: np.ndarray
    decel: np.ndarray
    accel_max: np.ndarray
    decel_max: np.ndarray
    signals: np.ndarray
    extent: np.ndarray
    edge_id: np.ndarray
    lane_id: np.ndarray
    lane_index: np.ndarray
    sumo_repr: bool = None

    def __init__(self, *states):
        self.actor_states: List[ActorState] = list(states)

        if len(states) == 0:
            return
        for field in fields(self):
            attr_name = field.name
            if attr_name == 'sumo_repr':
                continue

            accum_vals = np.array([getattr(actor_state, attr_name) for actor_state in self.actor_states])
            setattr(self, attr_name, accum_vals)

    def to_universal(self):
        states_universal = [state.to_universal() for state in self.actor_states]
        self.sumo_repr = False
        return TrafficState(*states_universal)

    def to_sumo(self):
        states_sumo = [state.to_sumo() for state in self.actor_states]
        self.sumo_repr = True
        return TrafficState(*states_sumo)

    def relative_to(self, context_state: ActorState):
        # TODO: could be improved by calculating relative values on traffic attribute lists/arrays directly
        states_rel = [state.relative_to(context_state) for state in self.actor_states]
        return TrafficState(*states_rel)

    def distances_to(self, context_state: ActorState) -> np.ndarray:
        d = np.array([state.distance_to(context_state) for state in self.actor_states])
        return d

    def get_actor_state(self, actor_id):
        if actor_id not in self:
            raise ValueError
        return self[actor_id]

    def set_actor_state(self, actor_id, state):
        if actor_id not in self:
            raise ValueError
        self[actor_id] = state

    def index(self, item: Union[str, ActorState]):
        if not isinstance(item, str) and not isinstance(item, ActorState):
            raise ValueError

        veh_id = item if isinstance(item, str) else item.veh_id
        index = np.where(self.veh_id == veh_id)[0]
        if index.size == 0:
            raise ValueError
        return int(index)

    def __len__(self):
        return len(self.actor_states)

    def __contains__(self, item):
        if not isinstance(item, str) and not isinstance(item, ActorState):
            raise ValueError

        veh_id = item if isinstance(item, str) else item.veh_id
        return int(len(np.where(self.veh_id == veh_id))) > 0

    def __iter__(self):
        return (self[i] for i in range(len(self)))

    def __getitem__(self, key):
        if isinstance(key, slice):
            return TrafficState(*self.actor_states[key])
        elif isinstance(key, int):
            if key >= len(self):
                raise IndexError
            return deepcopy(self.actor_states[key])
        elif isinstance(key, str):
            if key not in self:
                raise KeyError
            return deepcopy(self.actor_states[self.index(key)])
        elif isinstance(key, List) or isinstance(key, np.ndarray):
            key = key.tolist() if isinstance(key, np.ndarray) else key
            str_query = [isinstance(x, str) for x in key]
            if any(str_query) and not all(str_query):
                raise KeyError
            if all(str_query):
                key = [self.index(x) for x in key]  # Convert to indices
            return TrafficState(*list(map(self.actor_states.__getitem__, key)))
        else:
            raise TypeError

    def __setitem__(self, key: Union[int, str, slice, List[str], List[int]], val: Union[ActorState, List[ActorState]]):
        if not isinstance(val, List) and not isinstance(val, ActorState):  # TrafficState would also be possible, but might be confusing
            raise ValueError

        actor_states = [val] if isinstance(val, ActorState) else val
        indices = None

        for field in fields(self):
            attr_name = field.name
            if attr_name == 'sumo_repr':
                continue

            attr_val = getattr(self, attr_name)

            if isinstance(key, List):
                str_query = [isinstance(x, str) for x in key]
                if any(str_query) and not all(str_query):
                    raise ValueError
                if all(str_query):
                    indices = [self.index(x) for x in key]  # Convert to indices
                for i, dest in enumerate(indices):
                    attr_val[dest] = getattr(actor_states[i], attr_name)
            else:
                if isinstance(key, slice):
                    indices = key
                elif isinstance(key, int):
                    if key >= len(self):
                        raise IndexError
                    indices = [key]
                elif isinstance(key, str):
                    if key not in self:
                        raise KeyError
                    indices = [self.index(key)]
                else:
                    raise KeyError

                for i, dest in enumerate(indices):
                    attr_val[dest] = getattr(actor_states[i], attr_name)

            setattr(self, attr_name, attr_val)

        # Update the internal self._states list
        if isinstance(indices, slice):
            self.actor_states[indices] = actor_states
        else:
            map(self.actor_states.__setitem__, indices, actor_states)

    def mask(self, actor_id: Union[str, List[str]]):
        if isinstance(actor_id, str): actor_id = [actor_id]
        indices = [i for i in range(len(self)) if self.veh_id[i] not in actor_id]
        return self[indices]

    def __delitem__(self, key):
        if isinstance(key, slice):
            key = np.arange(slice.start, slice.stop, slice.step)

        elif isinstance(key, int):
            if key < 0 or key >= len(self):
                raise IndexError
            key = [key]

        elif isinstance(key, str):
            if key not in self:
                raise KeyError
            key = [self.index(key)]

        elif isinstance(key, List):
            str_query = [isinstance(x, str) for x in key]
            if any(str_query) and not all(str_query):
                raise KeyError
            if all(str_query):
                key = [self.index(x) for x in key]  # Convert to indices
        else:
            raise TypeError

        for field in fields(self):
            attr_name = field.name
            if attr_name == 'sumo_repr':
                continue
            full_val = getattr(self, attr_name)
            if isinstance(full_val, List):
                full_val = del_list_indices(full_val, key)
            else:
                full_val = np.delete(full_val, key)
            setattr(self, attr_name, full_val)

        self.actor_states = del_list_indices(self.actor_states, key)


class SUMOActor:
    def __init__(self, actor_id: str, traci):
        self._state: ActorState = ActorState()
        self._actor_id: str = actor_id
        self._outdated: bool = True
        self.traci = traci

    def flag_outdated(self) -> None:
        self._outdated = True

    @property
    def state(self) -> ActorState:
        if not self._outdated:
            return self._state
        self._outdated = False

        results = self.traci.vehicle.getSubscriptionResults(self._actor_id)
        self._state = ActorState(
            veh_id=self._actor_id,
            veh_type=results[traci.constants.VAR_TYPE],
            veh_class=results[traci.constants.VAR_VEHICLECLASS],
            # color=results[traci.constants.VAR_COLOR],
            length=results[traci.constants.VAR_LENGTH],
            width=results[traci.constants.VAR_WIDTH],
            height=results[traci.constants.VAR_HEIGHT],
            location=np.array(list(results[traci.constants.VAR_POSITION3D])),
            rotation=np.array([results[traci.constants.VAR_SLOPE], results[traci.constants.VAR_ANGLE], 0.0]),
            lane_position_lat=results[traci.constants.VAR_LANEPOSITION_LAT],
            velocity=results[traci.constants.VAR_SPEED],
            velocity_lat=results[traci.constants.VAR_SPEED_LAT],
            velocity_min=None if self._state is None else self._state.velocity_min,  # We can't get this from SUMO engine
            velocity_max=results[traci.constants.VAR_MAXSPEED],
            accel=results[traci.constants.VAR_ACCELERATION],
            decel=results[traci.constants.VAR_APPARENT_DECEL],
            accel_max=results[traci.constants.VAR_ACCEL],
            decel_max=results[traci.constants.VAR_DECEL],
            signals=results[traci.constants.VAR_SIGNALS],
            extent=np.array([
                results[traci.constants.VAR_LENGTH] / 2.0,
                results[traci.constants.VAR_WIDTH] / 2.0,
                results[traci.constants.VAR_HEIGHT] / 2.0]),
            edge_id=results[traci.constants.VAR_ROAD_ID],
            lane_id=results[traci.constants.VAR_LANE_ID],
            lane_index=results[traci.constants.VAR_LANE_INDEX],
            sumo_repr=True,
        ).to_universal()

        return self._state

    @state.setter
    def state(self, new_state: ActorState) -> None:
        if new_state.sumo_repr != self._state.sumo_repr:
            if new_state.sumo_repr:
                new_state = new_state.to_universal()
            else:
                new_state = new_state.to_sumo()
        assert new_state.sumo_repr == self._state.sumo_repr

        new_state = deepcopy(new_state)
        new_state_sumo = new_state.to_sumo()
        own_state_sumo = deepcopy(self.state).to_sumo()

        for field in fields(new_state):
            attr_name: str = field.name
            old_val = getattr(self._state, attr_name)
            new_val = getattr(new_state, attr_name)

            # Check if something changed
            if new_val is None:
                continue
            elif isinstance(new_val, np.ndarray) and np.all(old_val == new_val):
                continue
            elif isinstance(new_val, float) or isinstance(new_val, str) and old_val == new_val:
                continue
            elif isinstance(new_val, List):  # This should not happen
                raise ValueError

            new_val_sumo = getattr(new_state_sumo, attr_name)
            # Do the TraCI calls
            if attr_name == 'veh_id':
                continue  # We can't modify the id of a SUMO vehicle
            if attr_name == 'veh_type':
                continue  # We can't modify the id of a SUMO vehicle
            elif attr_name == 'veh_class':
                self.traci.vehicle.setVehicleClass(self._actor_id, new_val_sumo)
            # elif attr_name == 'color':  # Does not work with libsumo
            #     self.traci.vehicle.setColor(self._actor_id, new_val_sumo)
            elif attr_name == 'length':
                self.traci.vehicle.setLength(self._actor_id, new_val_sumo)
            elif attr_name == 'width':
                self.traci.vehicle.setWidth(self._actor_id, new_val_sumo)
            elif attr_name == 'height':
                self.traci.vehicle.setHeight(self._actor_id, new_val_sumo)
                pass
            elif attr_name == 'location' or attr_name == 'rotation':
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
                location = new_state_sumo.location if new_state_sumo.location is not None else own_state_sumo.location
                rotation = new_state_sumo.rotation if new_state_sumo.rotation is not None else own_state_sumo.rotation
                edge_id = new_state_sumo.edge_id if new_state_sumo.edge_id is not None else ""
                lane_index = new_state_sumo.lane_index if new_state_sumo.lane_index is not None else -1
                self.traci.vehicle.moveToXY(
                    self._actor_id, edge_id, lane_index, location[0], location[1], angle=rotation[1], keepRoute=1, matchThreshold=200.0,
                )
            elif attr_name == 'velocity':
                self.traci.vehicle.setSpeed(self._actor_id, new_val_sumo)
            elif attr_name == 'velocity_lat':
                pass  # Not available
            elif attr_name == 'velocity_max':
                self.traci.vehicle.setMaxSpeed(self._actor_id, new_val_sumo)
            elif attr_name == 'accel':
                self.traci.vehicle.setAcceleration(self._actor_id, new_val_sumo)
            elif attr_name == 'decel':
                self.traci.vehicle.setApparentDecel(self._actor_id, new_val_sumo)
            elif attr_name == 'accel_max':
                self.traci.vehicle.setAccel(self._actor_id, new_val_sumo)
            elif attr_name == 'decel_max':
                self.traci.vehicle.setDecel(self._actor_id, new_val_sumo)
            elif attr_name == 'signals':
                self.traci.vehicle.setSignals(self._actor_id, new_val_sumo)
            elif attr_name == 'extent':
                pass  # Covered by width, length and height
            elif attr_name == 'edge_id':
                pass  # Implicitly covered by traci.vehicle.moveToXY
            elif attr_name == 'lane_id':
                pass
            elif attr_name == 'lane_index':
                pass

            # Set new value inside self._state
            setattr(self._state, attr_name, new_val)


class TrafficManager:
    def __init__(self, config):
        self._config = config
        self.traci = None
        self.actors: Optional[List[SUMOActor]] = None
        self.actor_ids: Optional[List[str]] = None
        self.actor_states: Optional[List[ActorState]] = None
        self._traffic_state: Optional[TrafficState] = None
        self._subscribed_context = False

    def reset(self, traci) -> None:
        self.actors = []
        self.actor_ids = []
        self.actor_states = None
        self._traffic_state = None
        self.traci = traci

    def step(self) -> None:
        spawned_actors = set(self.traci.simulation.getDepartedIDList())
        destroyed_actors = set(self.traci.simulation.getArrivedIDList())
        duplicates = set(spawned_actors) & set(destroyed_actors)
        for dup in duplicates:
            spawned_actors.remove(dup)
            destroyed_actors.remove(dup)

        for actor_id in spawned_actors:  # Subscribe to new actors
            self.subscribe(actor_id)
            self.actor_ids.append(actor_id)
            self.actors.append(SUMOActor(actor_id, self.traci))
            if actor_id == self._config.simulation.egoID:  # This is needed for TrafficManager.get_ego_context() method
                import traci
                self.traci.vehicle.subscribeContext(self._config.simulation.egoID, traci.constants.CMD_GET_VEHICLE_VARIABLE, dist=100.0)
                self.traci.vehicle.addSubscriptionFilterTurn(downstreamDist=100.0, foeDistToJunction=30.0)

        # Delete de-spawned ones
        del_indices: List[int] = []
        for actor_id in destroyed_actors:
            if actor_id in self.actor_ids:
                # try:
                if actor_id in self.traci.vehicle.getIDList():
                    self.unsubscribe(actor_id)  # TODO: Don't know why we need this here
                # except traci.exceptions.TraCIException:
                #    pass
                index = self.actor_ids.index(actor_id)
                del_indices.append(index)
        self.actors, self.actor_ids = list(map(del_list_indices, [self.actors, self.actor_ids], [del_indices] * 2))  # Remove the data entries

        [actor.flag_outdated() for actor in self.actors]  # Should be recalculated
        for actor in self.actors:
            actor.traci = self.traci

        self.actor_states = None
        self._traffic_state = None

    @property
    def traffic_state(self) -> Optional[TrafficState]:
        if self._traffic_state is not None:
            return self._traffic_state
        elif self.actors is None:
            return None

        self.actor_states = [actor.state for actor in self.actors]
        self._traffic_state = TrafficState(*self.actor_states).to_universal()  # Querying from the TM returns in universal format
        return deepcopy(self._traffic_state)

    @traffic_state.setter
    def traffic_state(self, x: TrafficState) -> None:
        if not all([self._traffic_state[i].veh_id == x[i].veh_id] for i in range(len(x))):
            raise ValueError

        self._traffic_state = x
        self.actor_states = x.actor_states
        map(setattr, self.actors, ['state'] * len(x), self.actor_states)  # Broadcasts changes to SUMO

    def get_actor_state(self, actor_id: str) -> ActorState:
        return deepcopy(self.traffic_state[actor_id])

    def set_actor_state(self, x: ActorState) -> ActorState:
        for field in fields(x):  # Convert to correct types
            if field.type == np.ndarray and not isinstance(getattr(x, field.name), np.ndarray):
                setattr(x, field.name, np.array(getattr(x, field.name)))

        actor_id = x.veh_id
        actor_index = self.actor_ids.index(actor_id)
        self.actors[actor_index].state = x.to_universal()  # Update the SUMOActor, who does the TraCI calls
        self._traffic_state[actor_index] = self.actors[actor_index].state  # Also, update the TrafficState object
        return deepcopy(self.actors[actor_index].state)

    def get_junction_foes(self):
        ego_actor = self.actors[self.actor_ids.index(self._config.simulation.egoID)]

    def subscribe(self, actor_id):
        self.traci.vehicle.subscribe(actor_id, [
            traci.constants.VAR_TYPE,
            traci.constants.VAR_VEHICLECLASS,
            traci.constants.VAR_COLOR,
            traci.constants.VAR_LENGTH,
            traci.constants.VAR_WIDTH,
            traci.constants.VAR_HEIGHT,
            traci.constants.VAR_POSITION3D,
            traci.constants.VAR_ANGLE,
            traci.constants.VAR_SLOPE,
            traci.constants.VAR_LANEPOSITION_LAT,
            traci.constants.VAR_SPEED,
            traci.constants.VAR_SPEED_LAT,
            traci.constants.VAR_MAXSPEED,
            traci.constants.VAR_SIGNALS,
            traci.constants.VAR_ACCEL,
            traci.constants.VAR_DECEL,
            traci.constants.VAR_ACCELERATION,
            traci.constants.VAR_APPARENT_DECEL,
            traci.constants.VAR_ROAD_ID,
            traci.constants.VAR_LANE_ID,
            traci.constants.VAR_LANE_INDEX,
            traci.constants.VAR_ROUTE_ID,
        ])

    def unsubscribe(self, actor_id):
        self.traci.vehicle.unsubscribe(actor_id)

    def get_ego_context(self):
        ret = {
            'leaders': [],
            'followers': [],
            'foes': [],
        }

        if self._config.simulation.egoID not in self.actor_ids:
            return ret

        egoID = self._config.simulation.egoID
        ret['foes'] += self.traci.vehicle.getContextSubscriptionResults(egoID).keys()
        ret['followers'] += [x[0] for x in self.traci.vehicle.getRightFollowers(egoID)]
        ret['followers'] += [x[0] for x in self.traci.vehicle.getLeftFollowers(egoID)]
        follower = self.traci.vehicle.getFollower(egoID)
        ret['followers'] += [follower[0]] if follower is not None and follower[0] != '' else []
        ret['leaders'] += [x[0] for x in self.traci.vehicle.getRightLeaders(egoID)]
        ret['leaders'] += [x[0] for x in self.traci.vehicle.getLeftLeaders(egoID)]
        leader = self.traci.vehicle.getLeader(egoID)
        ret['leaders'] += [leader[0]] if leader is not None and leader[0] != '' else []
        return ret


if __name__ == '__main__':
    actor_states = [ActorState(veh_id=f'actor{i}') for i in range(10)]
    traffic_state = TrafficState(*actor_states)
    print(traffic_state, '\n')
    print(traffic_state[1], '\n')
    print(traffic_state['actor2'], '\n')
    print(traffic_state[[1, 2]], '\n')
    print(traffic_state[['actor3', 'actor4']], '\n')

    traffic_state2 = traffic_state[0:5]
    print(traffic_state2, '\n')
    del traffic_state2[0]
    print(traffic_state2, '\n')
    del traffic_state2['actor2']
    print(traffic_state2, '\n')

    traffic_state2[0] = traffic_state[-1]
    print(traffic_state2)

    print(traffic_state, '\n')

    traffic_state2 = deepcopy(traffic_state)
    del traffic_state2[0]
    print(traffic_state)
