import shapely.geometry
import shapely.affinity
import numpy as np


class RotatedRect:
    """A rotated rectangle. Used to detect collisions using the `shapely` Python package."""
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


def check_collision(veh_id, traffic_manager):
    """Check for collisions between ego and non-ego vehicle."""
    ego_id = veh_id
    traffic_state = traffic_manager.traffic_state
    ego_state = traffic_state[ego_id]
    traffic_state = traffic_state.mask(ego_id)

    dist = traffic_state.distances_to(ego_state)
    radius_mask = np.argwhere(dist <= 20.0).flatten()
    if dist.size == 0 or radius_mask.size == 0:
        return False

    traffic_state = traffic_state[radius_mask]

    def create_rect(state):
        return RotatedRect(
            *state.location[:2],
            state.width,
            state.length,
            state.rotation[1]
        )

    ego_rect = create_rect(ego_state)
    traffic_rects = list(map(create_rect, traffic_state))
    inter = list(map(ego_rect.intersection, traffic_rects))
    return not all([x.is_empty for x in inter])
