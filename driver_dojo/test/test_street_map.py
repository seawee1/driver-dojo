import unittest
from typing import List
import sumolib
import os
from os.path import join, dirname

from driver_dojo.common.road_manager import StreetMap


class DummyBasicScenario:
    def __init__(self, sumo_net, route_edges):
        self.sumo_net = sumo_net
        self.route_edges = route_edges


class TestStreetMap(unittest.TestCase):
    net: sumolib.net.Net = None
    route_edges: List[str] = None

    @classmethod
    def setUpClass(self):
        root_path = dirname(__file__)
        asset_path = join(root_path, 'assets', 'test_street_map')
        self.net = sumolib.net.readNet(join(asset_path, 'net.net.xml'), withInternal=True)

        for route in sumolib.xml.parse(join(asset_path, 'demand.rou.xml'), "route"):
            if route.id == 'route_ego':
                self.route_edges = route.edges.split()
        self.scenario = DummyBasicScenario(self.net, self.route_edges)

    def runTest(self):
        street_map = StreetMap()
        street_map.reset(self.scenario)

        import matplotlib.pyplot as plt
        for lane_id, lane in street_map.graph.roads.items():
            plt.plot(lane.shape.left_border[:, 0], lane.shape.left_border[:, 1])
            plt.plot(lane.shape.right_border[:, 0], lane.shape.right_border[:, 1])
        plt.show()
        for lane_id, lane in street_map.graph.route_partition.roads.items():
            plt.plot(lane.shape.left_border[:, 0], lane.shape.left_border[:, 1])
            plt.plot(lane.shape.right_border[:, 0], lane.shape.right_border[:, 1])
        plt.show()

