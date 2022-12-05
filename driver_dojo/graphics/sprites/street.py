from typing import Union, List

import pyglet

ROAD_COLOR = (128, 128, 128)
INNER_LANE_MARKING_COLOR = (255, 255, 255)
LANE_SHAPE_COLOR = (0, 255, 0)
BETWEEN_ROAD_MARKING_COLOR = (0, 0, 0)
OUTER_ROAD_MARKING_COLOR = (0, 0, 0)


def draw_multi_line(line_points, color, batch, group, width=0.5):
    shapes = []
    for i in range(len(line_points) - 1):
        j = i + 1
        x, y = line_points[i]
        x2, y2 = line_points[j]
        shapes.append(
            pyglet.shapes.Line(x, y, x2, y2, width=width, color=color, batch=batch, group=group)
        )
    return shapes


class LaneSprite:
    def __init__(self, street_map, lane_id, batch, group_marking, group_road):
        self.street_map = street_map
        self._lane_id = lane_id
        self._lane_node = self.street_map.graph.lanes[lane_id]
        self._batch = batch
        self._group_road = group_road
        self._group_marking = group_marking

        self._shapes: List[
            Union[
                pyglet.shapes.Polygon,
                pyglet.shapes.Line,
            ]
        ] = []
        self._init_shapes()

    def _init_shapes(self):
        # self._shapes.append(  # TODO: doesn't work this way
        #     pyglet.shapes.Polygon(*self._lane_node.polygon, color=ROAD_COLOR, batch=self._batch, group=self._group_road)
        # )

        self._shapes += draw_multi_line(self._lane_node.shape.shape, ROAD_COLOR, self._batch, self._group_marking, self._lane_node.width)

        if self._lane_node.left_neigh is None:  # Between left and right road
            self._shapes += draw_multi_line(self._lane_node.shape.left_border, BETWEEN_ROAD_MARKING_COLOR, self._batch, self._group_marking)
        if self._lane_node.right_neigh is not None:  # Between lanes of one driving direction
            self._shapes += draw_multi_line(self._lane_node.shape.right_border, INNER_LANE_MARKING_COLOR, self._batch, self._group_marking)

    def delete(self):
        [x.delete() for x in self._shapes]


class NodeSprite:
    def __init__(self, street_map, node_id, batch, group_marking, group_road):
        self.street_map = street_map
        self._node_id = node_id
        self._node = self.street_map.graph.junctions[node_id]
        self._batch = batch
        self._group_road = group_road
        self._group_marking = group_marking
        self._shapes: List[
            Union[
                pyglet.shapes.Polygon,
                pyglet.shapes.Line,
            ]
        ] = []

        self._init_shapes()

    def _init_shapes(self):
        self._shapes.append(
            pyglet.shapes.Polygon(*self._node.shape, color=ROAD_COLOR, batch=self._batch, group=self._group_road)
        )

    def delete(self):
        [x.delete() for x in self._shapes]
