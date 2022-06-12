import shapely.geometry
import shapely.affinity
import numpy as np

import driver_dojo.common.state_variables as state_variables


class RotatedRect:
    """A rotated rectangle. Used by the collision detection and the renderer."""

    def __init__(self, cx, cy, w, h, angle):
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
        self.angle = np.degrees(angle)

    def get_contour(self):
        w = self.w
        h = self.h
        c = shapely.geometry.box(0.0, 0.0, h, w)
        c = shapely.affinity.translate(c, -h / 2.0, -w / 2.0)
        point = shapely.geometry.Point(0.0, 0.0)
        rc = shapely.affinity.rotate(c, self.angle, origin=point)
        return shapely.affinity.translate(rc, self.cx, self.cy)

    def intersection(self, other):
        return self.get_contour().intersection(other.get_contour())


def check_collision():
    """Check for collisions between ego and non-ego vehicle."""

    dists = state_variables.traffic_manager.distance_to_ego
    if dists is None:
        return False

    mask_radius = np.argwhere(dists <= 20.0).flatten()
    if mask_radius.shape[0] == 0:
        return False

    traffic_state = state_variables.traffic_manager.traffic_state_transformed()
    traffic_state = traffic_state[mask_radius]

    ego_state = state_variables.traffic_manager.ego_state_transformed
    ego_rect = RotatedRect(
        *ego_state("position"),
        ego_state("width"),
        ego_state("length"),
        ego_state("angle")
    )

    index = state_variables.traffic_manager.index
    for t in traffic_state:
        traffic_rect = RotatedRect(
            t[index["position"][0]],
            t[index["position"][1]],
            t[index["width"]],
            t[index["length"]],
            t[index["angle"]],
        )
        if not ego_rect.intersection(traffic_rect).is_empty:
            # Debug
            # print('Traffic:', t[index["position"][0]], t[index["position"][1]], t[index["width"]], t[index["length"]], t[index["angle"]])
            # print('Ego:', *ego_state("position"), ego_state("width"), ego_state("length"), ego_state("angle"))
            # import matplotlib.pyplot as plt
            # x, y = ego_rect.get_contour().exterior.xy
            # plt.plot(x, y)
            # x, y = traffic_rect.get_contour().exterior.xy
            # plt.plot(x, y)
            # plt.gca().set_aspect('equal', adjustable='box')
            # plt.show()
            return True
    return False
