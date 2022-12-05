from dataclasses import dataclass, fields, field
from typing import List, Tuple, Optional
import xml.etree.ElementTree as ET

from driver_dojo.core.types import SpeedClass

@dataclass
class Base:
    def as_dict(self):
        ret = {}
        for f in fields(self):
            val = getattr(self, f.name)
            if val is None:
                continue
            if f.name == 'name':
                ret['id'] = val
            elif f.name == 'src':
                ret['from'] = val
            elif f.name == 'pas':
                ret['pass'] = val
            elif f.name == 'shape':
                ret['shape'] = ' '.join([f'{x},{y}' for x, y in zip(val[0], val[1])])
            elif f.name == 'priority':
                ret['priority'] = int(val)
            else:
                ret[f.name] = val
        for k, v in ret.items():
            ret[k] = str(v)
        return ret

@dataclass
class Edge(Base):
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
    rightOfWay: str = field(default='default')
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


def write_plain_xml(
    path: str,
    nodes: Tuple[Node],
    edges: Tuple[Edge],
    conns: Optional[Tuple[Connection]] = None
):
    def write(parent_name, child_name, elems, file_name):
        root = ET.Element(parent_name)
        for elem in elems:
            child = ET.SubElement(root, child_name, attrib=elem.as_dict())

        tree = ET.ElementTree(root)
        with open(file_name, 'wb') as f:
            tree.write(f)

    write('nodes', 'node', nodes, f'{path}.nod.xml')
    write('edges', 'edge', edges, f'{path}.edg.xml')
    if conns is not None:
        write('connections', 'connection', conns, f'{path}.con.xml')

    import time
    time.sleep(1.0)

    return f'{path}.nod.xml', f'{path}.edg.xml', (f'{path}.con.xml' if conns is not None else None)

