import numpy as np
import shapely.geometry
from shapely.geometry import Point, LineString
import matplotlib.pyplot as plt
import logging

from driver_dojo.common import state_variables
from driver_dojo.observer import BaseObserver
from driver_dojo.common.utils import (
    net_to_polygons,
    normalize_observations,
    create_observation_space,
)


class RoadShapeObserver(BaseObserver):
    def __init__(self):
        super().__init__()
        self.num_rays = state_variables.config["observations"]["rs_num_rays"]
        self.ray_dist = state_variables.config["observations"]["rs_ray_dist"]
        self.opening_angle = state_variables.config["observations"]["rs_opening_angle"]
        self.num_inters = state_variables.config["observations"]["rs_num_inter_per_ray"]
        self.relative = state_variables.config["observations"]["relative_to_ego"]
        self.network_polygon = None
        # self.debug = state_variables.config['debug']['road_shape_observer']

        if self.relative:
            x_low, y_low, x_high, y_high = (
                -self.ray_dist,
                -self.ray_dist,
                self.ray_dist,
                self.ray_dist,
            )
        else:
            x_low, y_low, x_high, y_high = state_variables.net_bbox

        self.low = np.array(
            [x_low, y_low] * self.num_rays * self.num_inters, dtype=np.float32
        )
        self.high = np.array(
            [x_high, y_high] * self.num_rays * self.num_inters, dtype=np.float32
        )

        self.observation_space = create_observation_space(
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )

        # if self.debug:
        #     plt.gca().set_aspect('equal', adjustable='box')

    def observe(self):
        ego = state_variables.vehicle
        ego_pos = ego.position
        ego_angle = ego.angle
        ray_angles = np.linspace(
            ego_angle - np.radians(self.opening_angle / 2.0),
            ego_angle + np.radians(self.opening_angle / 2.0),
            num=self.num_rays,
            endpoint=True,
        )[::-1]
        ray_vectors = np.vstack([np.cos(ray_angles), np.sin(ray_angles)]).T

        ego_point = Point(*ego_pos)
        intersection_points = []
        intersection_xy = np.zeros((self.num_inters * self.num_rays, 2))
        for i, vec in enumerate(ray_vectors):
            ray_pos = ego_pos + self.ray_dist * vec
            ray_point = Point(*ray_pos)
            ray_line = LineString([ego_point, ray_point])

            # if self.debug:
            #     xs, ys = zip((ego_pos[0], ego_pos[1]), (ray_pos[0], ray_pos[1]))
            #     plt.plot(xs, ys)

            intersection = []

            # We either just calculate ray intersections for one polygon or iterate over a list of geometries from a MultiPolygon
            if isinstance(self.network_polygon, shapely.geometry.MultiPolygon):
                iter_obj = [geom.exterior for geom in self.network_polygon.geoms]
                #logging.warning(
                    #f"RoadShapeObserver has MultiGeometry as base. This shouldn't happen, but is also taken care off. "
                    #f"Level-seed: {state_variables.level_seed}."
                #)
            else:
                iter_obj = [self.network_polygon.exterior]

            # Calculate the intersection points by iterating over the polygon(s)
            for geom in iter_obj:
                partial_inter = geom.intersection(ray_line)

                if partial_inter.is_empty:
                    pass
                elif (
                    partial_inter.geom_type.startswith("Multi")
                    or partial_inter.geom_type == "GeometryCollection"
                ):
                    for inter in partial_inter.geoms:
                        intersection.append(inter)
                else:
                    intersection.append(partial_inter)

            # Calculate xy position of the intersections and build the intersection array for that ray
            if len(intersection) == 0:
                intersection_points.extend([None] * self.num_inters)
            else:
                dists = []
                for inter in intersection:
                    dists.append(inter.distance(ego_point))
                sort_idx = np.argsort(np.array(dists))
                for j in range(self.num_inters):
                    # We don't have an intersection for that index
                    if j >= sort_idx.shape[0]:
                        intersection_points.append(None)
                    # We have one
                    else:
                        intersection_xy[i * self.num_inters + j, 0] = intersection[
                            sort_idx[j]
                        ].x
                        intersection_xy[i * self.num_inters + j, 1] = intersection[
                            sort_idx[j]
                        ].y
                        intersection_points.append(intersection[sort_idx[j]])

            # if intersection.is_empty:
            #     intersection_points.extend([None] * self.num_inters)
            #
            # elif (
            #     intersection.geom_type.startswith("Multi")
            #     or intersection.geom_type == "GeometryCollection"
            # ):
            #     dists = []
            #     for shp in intersection.geoms:
            #         dists.append(shp.distance(ego_point))
            #
            #     sort_idx = np.argsort(np.array(dists))
            #     for j in range(self.num_inters):
            #         if sort_idx.shape[0] <= j:
            #             intersection_points.append(None)
            #         else:
            #             intersection_xy[
            #                 i * self.num_inters + j, 0
            #             ] = intersection.geoms[sort_idx[j]].x
            #             intersection_xy[
            #                 i * self.num_inters + j, 1
            #             ] = intersection.geoms[sort_idx[j]].y
            #             intersection_points.append(intersection.geoms[sort_idx[j]])
            # else:
            #     intersection_points.append(intersection)
            #     intersection_points.extend([None] * (self.num_inters - 1))
            #     intersection_xy[i * self.num_inters, 0] = intersection.x
            #     intersection_xy[i * self.num_inters, 1] = intersection.y

        no_inter_idx = np.array(
            [i for i, x in enumerate(intersection_points) if x is None]
        )
        if self.relative:
            intersection_xy = intersection_xy - ego_pos
            theta = -state_variables.vehicle.angle + np.radians(90)
            rot_mat = np.array(
                [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
            )
            intersection_xy = np.dot(rot_mat, intersection_xy.T).T

        if state_variables.config["debug"]["plot_road_shape_observations"]:
            if (
                state_variables.time_step
                % state_variables.config["debug"]["plot_road_shape_observations"]
                == 0
            ):
                xs, ys = zip(*self.network_polygon.exterior.coords)
                plt.plot(xs, ys)

                for inter in intersection_points:
                    if inter is None:
                        continue
                    x, y = inter.x, inter.y
                    plt.scatter(x, y)

                plt.draw()
                plt.pause(0.0001)
                plt.clf()

        obs = normalize_observations(
            intersection_xy.flatten(),
            self.low,
            self.high,
            state_variables.config["observations"]["feature_scaling"],
        )
        if no_inter_idx.shape[0] > 0:
            obs[no_inter_idx * 2] = 0.0
            obs[no_inter_idx * 2 + 1] = 0.0

        assert obs.shape == self.low.shape

        return obs

    def reset(self):
        self.network_polygon = net_to_polygons(
            state_variables.net, fuse_lanes=True, fuse_all=True
        )[0]
