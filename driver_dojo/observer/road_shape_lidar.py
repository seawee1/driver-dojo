from typing import Tuple
import numpy as np
from shapely import affinity
from shapely.geometry import Point, MultiLineString, GeometryCollection, MultiPoint, LineString

from driver_dojo.observer import BaseObserver


class RoadShapeLidarObserver(BaseObserver):
    def __init__(self, config, vehicle, traffic_manager, street_map):
        super().__init__(config, vehicle, traffic_manager)
        self._street_map = street_map
        self._num_rays = self.config.observations.rs_num_rays
        self._ray_dist = self.config.observations.rs_ray_dist
        self._opening_angle = self.config.observations.rs_opening_angle
        self._num_inters = self.config.observations.rs_num_inter_per_ray
        self._relative = self.config.observations.relative_to_ego
        self._network_linestring = None  # TODO
        self._ray_linestring = None
        self.ray_linestring_cur = None  # Used for debugging inside driver_dojo.env.Env
        self._build_ray_linestring()

        if self._relative:
            x_low, y_low, x_high, y_high = (
                -self._ray_dist,
                -self._ray_dist,
                self._ray_dist,
                self._ray_dist,
            )
        else:
            x_low, y_low, x_high, y_high = -np.inf, -np.inf, np.inf, np.inf

        num_inters_max = self._num_rays * self._num_inters
        self.low = np.array([x_low, y_low] * num_inters_max, dtype=np.float64)
        self.high = np.array([x_high, y_high] * num_inters_max, dtype=np.float64)

    def step(self):
        ego = self.vehicle
        ego_pos = ego.location[:2]
        ego_angle = ego.rotation[1]
        ego_point = Point(ego_pos)

        ray_linestring = affinity.rotate(self._ray_linestring, ego_angle, origin=Point(0.0, 0.0), use_radians=True)
        ray_linestring = affinity.translate(ray_linestring, xoff=ego_pos[0], yoff=ego_pos[1])
        self._ray_linestring_cur = ray_linestring

        def create_data_tuple(p: Point) -> Tuple[float, np.ndarray]:
            """Computes tuple from intersection point to be appended to inters_accum."""
            d = p.distance(ego_point)
            if d > self._ray_dist:
                return None
            return d, np.array([p.x, p.y])

        inters_accum = []
        obs = np.zeros((self._num_rays, self._num_inters, 2))
        for i, ray in enumerate(ray_linestring.geoms):
            inters_accum.append([])
            for border in self._network_linestring.geoms:
                inters = ray.intersection(border)
                if inters.is_empty:
                    continue
                elif isinstance(inters, Point):  # One intersection
                    tup = create_data_tuple(inters)
                    if tup is None:
                        continue
                    inters_accum[-1].append(tup)
                elif isinstance(inters, MultiPoint):  # Multiple intersections
                    for point in inters.geoms:
                        tup = create_data_tuple(point)
                        if tup is None:
                            continue
                        inters_accum[-1].append(tup)
                elif isinstance(inters, GeometryCollection):  # Mixed intersection types
                    for geom in inters.geoms:
                        if isinstance(geom, Point):  # Only interested in points
                            tup = create_data_tuple(geom)
                            if tup is None:
                                continue
                            inters_accum[-1].append(tup)
            inters_accum[-1].sort(key=lambda x: x[0])  # Sort by distance
            inters_accum[-1] = inters_accum[-1][:self._num_inters]  # Throw away too far away intersections
            for j, inter in enumerate(inters_accum[-1]):
                obs[i, j] = inter[1]

        if self.config.observations.relative_to_ego:  # Subtract ego position, rotate, swap axes
            obs = obs.reshape((-1, 2))
            obs -= ego_pos
            theta = -ego_angle + np.radians(90)
            rot = np.array(
                [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
            )
            obs = np.dot(rot, obs.T).T

        empty_mask = np.argwhere(obs.flatten() == 0.0)  # TODO: Not too problematic, but we assume intersection coords to have x and/or y != 0.0
        obs = self._normalize_obs(obs.flatten())  # Normalize
        obs[empty_mask] = 0.0
        return obs

    def reset(self):
        super().reset()
        self._build_network_linestring()

    def _build_network_linestring(self):
        lane_graph = self._street_map.graph.lanes
        nodes = self._street_map.graph.junctions

        lines = []
        for lane_id, lane_node in lane_graph.items():
            if lane_node.function == 'internal':
                continue
            if lane_node.left_neigh is None:  # Outer line/seperator line between opposing roads
                lines.append(lane_node.shape.left_border)
            if lane_node.right_neigh is None:  # Outer line
                lines.append(lane_node.shape.right_border)

        while True:  # Remove overlapping lines/duplicates, which is the case for lines between opposing lanes
            remove_idx = None
            for i, line1 in enumerate(lines):
                for j, line2 in enumerate(lines):
                    if i == j:
                        continue
                    if not LineString(line1).intersection(LineString(line2)).is_empty:
                        remove_idx = i
                        break
                if remove_idx is not None:
                    break
            if remove_idx is not None:
                del lines[remove_idx]
            else:
                break

        for _, node in nodes.items():  # Add outer contour of junctions
            lines.append(node.shape)

        self._network_linestring = MultiLineString(lines)

    def _build_ray_linestring(self):
        ray_angles = np.linspace(
            np.radians(-self._opening_angle / 2.0),
            np.radians(self._opening_angle / 2.0),
            num=self._num_rays,
            endpoint=True)

        lines = np.zeros((self._num_rays, 2, 2))
        lines[:, 1] = np.vstack([np.cos(ray_angles), np.sin(ray_angles)]).T * self._ray_dist
        self._ray_linestring = MultiLineString(lines.tolist())

    def explain(self):
        # ray_angles = np.linspace(
        #     np.radians(-self._opening_angle / 2.0),
        #     np.radians(self._opening_angle / 2.0),
        #     num=self._num_rays,
        #     endpoint=True)

        return ['x', 'y'] * self._num_inters * self._num_rays  # TODO: Add ray angles
