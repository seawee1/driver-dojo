import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, fields, asdict
from typing import Optional, Tuple, List, Dict
import pyclothoids
from pyclothoids import Clothoid

import numpy as np

from driver_dojo.core.types import SpeedClass


@dataclass
class Base:
    def as_dict(self):
        ret = {}
        for field in fields(self):
            val = getattr(self, field.name)
            if val is None:
                continue
            if field.name == 'name':
                ret['id'] = val
            elif field.name == 'src':
                ret['from'] = val
            elif field.name == 'pas':
                ret['pass'] = val
            elif field.name == 'shape':
                ret['shape'] = ' '.join([f'{x},{y}' for x, y in zip(val[0], val[1])])
            else:
                ret[field.name] = val
        for k, v in ret.items():
            ret[k] = str(v)
        return ret


@dataclass
class Edge(Base):
    # <edge id="<STRING>" from="<NODE_ID>" to="<NODE_ID>" [type="<STRING>"] [numLanes="<INT>"] [speed="<FLOAT>"] [priority="<UINT>"] [length="<FLOAT>"] [shape="<2D-POSITION>[ <2D-POSITION>]*"] [spreadType="center"] [allow="<VEHICLE_CLASS>[ <VEHICLE_CLASS>]*"] [disallow="<VEHICLE_CLASS>[ <VEHICLE_CLASS>]*"]/>
    name: str
    src: str
    to: str
    numLanes: Optional[int] = field(default=1)
    speed: Optional[float] = field(default=SpeedClass.Urban.value)
    priority: Optional[int] = field(default=1)
    shape: Optional[List[Tuple[float, float]]] = field(default=None)


@dataclass
class Node(Base):
    name: str
    x: float
    y: float
    z: float
    # enum ( "priority", "traffic_light", "right_before_left", "left_before_right", "unregulated", "priority_stop", "traffic_light_unregulated", "allway_stop", "rail_signal",
    # "zipper", "traffic_light_right_on_red", "rail_crossing")
    type: str = field(default=None)
    radius: float = field(default=None)
    keepClear: bool = field(default=None)
    shape: List[Tuple[float, float]] = field(default=None)


@dataclass
class Connection(Base):
    src: str
    trg: str
    fromLane: int
    toLane: int
    pas: bool = field(default=False)
    keepClear: bool = field(default=True)
    contPos: float = field(default=-1)
    visibility: float = field(default=4.5)
    speed: float = field(default=-1)
    shape: List[Tuple[float, float]] = field(default=None)
    # changeLeft
    # changeRight


class SumoPlainXML:
    def __init__(self):
        self.edges: Dict[str, Edge] = {}
        self.nodes: Dict[str, Edge] = {}
        self.conns: Dict[str, Edge] = {}

    def test(self):
        rng: np.random.RandomState = np.random.default_rng()

        shape_step = 1.0

        n_roads = rng.choice(
            [3, 4, 5]
        )

        n_lanes_in = [rng.choice([1, 2, 3], p=[0.4, 0.4, 0.2]) for i in range(n_roads)]
        n_lanes_out = [rng.choice([1, 2, 3], p=[0.4, 0.4, 0.2]) for i in range(n_roads)]
        road_sep = [rng.uniform(0.2, 2.0) if rng.uniform(0.0, 1.0) >= 0.5 else 0.0 for i in range(n_roads)]
        # if sum(n_lanes_in) == 0:
        #     n_lanes_in[rng.integers(0, n_roads)] = rng.choice([1, 2, 3])
        # if sum(n_lanes_out) == 0:
        #     n_lanes_out[rng.integers(0, n_roads)] = rng.choice([1, 2, 3])

        angle_var = 0.8 * (365.0 / n_roads / 2)
        road_angles = [
            np.radians(i * 365.0 / n_roads + rng.integers(-angle_var, angle_var)) for i in range(n_roads)
        ]
        t_inner = [rng.uniform(road_angle - np.pi/16, road_angle + np.pi/16) for road_angle in road_angles]
        t_outer = [rng.uniform(road_angle - np.pi/4, road_angle + np.pi/4) for road_angle in road_angles]

        airline_dist = [rng.integers(50.0, 100.0) for i in range(n_roads)]
        node_x, node_y = 0.0, 0.0

        self.nodes['center'] = Node('center', 0.0, 0.0, 0.0, type='priority')
        for i in range(n_roads):
            x, y = node_x + np.cos(road_angles[i]) * airline_dist[i], node_y + np.sin(road_angles[i]) * airline_dist[i]
            self.nodes[str(i)] = Node(str(i), x, y, 0.0)

        for i in range(n_roads):
            node = self.nodes[str(i)]
            cloth = Clothoid.G1Hermite(node_x, node_y, t_inner[i], node.x, node.y, t_outer[i])
            num_samples = int(cloth.length//shape_step)
            priority = 1
            if i == 0:
                priority = 2
            if n_lanes_in[i] > 0:
                shape = cloth.Reverse().SampleXY(num_samples)
                self.edges[f'{i}_in'] = Edge(f'{i}_in', str(i), 'center', numLanes=n_lanes_in[i], shape=shape, priority=priority)
            if n_lanes_out[i] > 0:
                if road_sep[i] > 0.0:
                    orth = road_angles[i] - np.pi/2
                    x_off, y_off = np.cos(orth) * road_sep[i], np.sin(orth) * road_sep[i]
                    cloth = cloth.Translate(x_off, y_off)
                shape = cloth.SampleXY(num_samples)
                self.edges[f'{i}_out'] = Edge(f'{i}_out', 'center', str(i), numLanes=n_lanes_out[i], shape=shape, priority=priority)

        # for edge_id, edge in self.edges:
        #     if 'out' in edge_id:
        #         continue
        #     edge_index = edge_id.split('_')[0]

        self.write_xml('nodes', 'node', self.nodes.values(), 'test1.nod.xml')
        self.write_xml('edges', 'edge', self.edges.values(), 'test1.edg.xml')

    def write_xml(self, parent_name, child_name, elems, file_name):
        root = ET.Element(parent_name)
        for elem in elems:
            child = ET.SubElement(root, child_name, attrib=elem.as_dict())

        tree = ET.ElementTree(root)
        with open(file_name, 'wb') as f:
            tree.write(f)

if __name__ == '__main__':
    s = SumoPlainXML()
    s.test()
