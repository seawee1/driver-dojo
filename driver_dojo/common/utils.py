import gym
import numpy as np
from shapely import geometry, affinity
from skimage.draw import polygon
from shapely.ops import unary_union
import matplotlib.pyplot as plt
from driver_dojo.core.types import FeatureScaling


def transform_to_sumo(x, y, yaw, front_d):
    """Transform from TUM to SUMO."""
    x_ = x + np.cos(yaw) * front_d
    y_ = y + np.sin(yaw) * front_d
    yaw_ = -np.degrees(yaw) + 90
    return x_, y_, yaw_


def transform_from_sumo(x, y, yaw, front_d):
    """Transform from SUMO to TUM."""
    yaw_ = np.radians(-yaw + 90)
    x_ = x - np.cos(yaw_) * front_d
    y_ = y - np.sin(yaw_) * front_d
    return x_, y_, yaw_


def wrap_to_pi(rad):
    """Wraps angle into interval [-pi, pi]."""
    return np.arctan2(np.sin(rad), np.cos(rad))


def interpolate_angles(rad1, rad2, alpha):
    """Linearly interpolates between `rad1` and `rad2` with `alpha`."""
    return np.arctan2(
        (1 - alpha) * np.sin(rad1) + alpha * np.sin(rad2),
        (1 - alpha) * np.cos(rad1) + alpha * np.cos(rad2),
    )


def rad_difference(source_rad, target_rad):
    """Finds the rotation necessary to match a target_rad"""
    return target_rad - source_rad


def vector2d_rad(x, y):
    """Calculates angle of a 2D vector in rad."""
    return np.arctan2(y, x)


def rotation_matrix(theta):
    """Returns a rotation matrix that rotates a vector of coordinates
    counterclockwise around the origin.
    
    Use as points_to_transform @ rotation_matrix(angle_in_radians).
    """
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def polygon_circle_shape(pos, radius, num_vert=15):
    x, y = pos
    circle_shape = []
    for rad in np.linspace(np.pi / 2, 2 * np.pi + np.pi / 2, num_vert + 1):
        vert_x = x + np.cos(rad) * radius
        vert_y = y + np.sin(rad) * radius
        vert = (vert_x, vert_y)
        circle_shape.append(vert)
    return circle_shape[::-1]


def net_to_polygons(net, fuse_lanes=True, fuse_all=False, debug_plt=False):
    # Edges first
    edges = net.getEdges(withInternal=False)
    polys = []
    for i, e in enumerate(edges):
        # Get all the lanes of an edge
        lanes = e.getLanes()
        # Create polygon for each lane
        lane_polys = []
        for lane in lanes:
            lane_polys.append(lane_to_polygon(lane.getShape(), lane.getWidth()))

        if fuse_lanes:
            # Dilate each lane polygon by 2 meter
            lane_polys = [
                lane_poly.buffer(1) for lane_poly in lane_polys if lane_poly.is_valid
            ]
            # Fuse all lanes into one polygon
            edge_poly = unary_union(lane_polys)
            if isinstance(edge_poly, geometry.Polygon):
                polys.append(edge_poly.buffer(-1))  # Erode again by 1 meter
            else:
                polys += [poly.buffer(-1) for poly in edge_poly.geoms]
        else:
            polys += [lane_poly for lane_poly in lane_polys]

    # Get the hull of each junction, create polygon
    nodes = net.getNodes()
    for n in nodes:
        if len(n.getShape()) < 3:
            continue
        polys.append(geometry.Polygon(n.getShape()))

    # Fuse into one polygon
    if fuse_all:
        # Dilate each lane polygon by 1 meter
        polys = [poly.buffer(1) for poly in polys if poly.is_valid]
        # Fuse all lanes into one polygon
        polys = [unary_union(polys)]
        polys = [poly.buffer(-1) for poly in polys if poly.is_valid]

    # Plot the street network polygons
    if debug_plt:
        for p in polys:
            xs, ys = zip(*p.exterior.coords)
            plt.plot(xs, ys)
        plt.show()

    return polys


def car_to_polygon(car_data):
    x_center, y_center = car_data[0], car_data[1]
    yaw_rad, length, width = car_data[2], car_data[5], car_data[6]
    poly_points = [
        (-length / 2.0, -width / 2.0),  # bottom left
        (length / 2.0, -width / 2.0),  # bottom right
        (length / 2.0, width / 2.0),  # top right
        (-length / 2.0, width / 2.0),  # top left
    ]

    # poly_points = [
    #     (x_center - length / 2.0, y_center - width / 2.0),
    #     (x_center + length / 2.0, y_center - width / 2.0),
    #     (x_center + length / 2.0, y_center + width / 2.0),
    #     (x_center - length / 2.0, y_center + width / 2.0),
    # ]
    car_polygon = geometry.Polygon(poly_points)

    car_polygon = affinity.rotate(car_polygon, np.rad2deg(yaw_rad), (0.0, 0.0))
    car_polygon = affinity.translate(car_polygon, x_center, y_center)
    return car_polygon


def lane_to_polygon(lane_points, width, left_right=False):
    # Store points of left and right side of lane inside this list
    left = []
    right = []

    for i in range(len(lane_points)):
        # We take mean normal of incoming segments for each shape point
        if i == 0:
            p1 = np.array(lane_points[i])
            p2 = np.array(lane_points[i + 1])
            direction = p2 - p1
        elif i == len(lane_points) - 1:
            p1 = np.array(lane_points[i - 1])
            p2 = np.array(lane_points[i])
            direction = p2 - p1
        else:
            p1 = np.array(lane_points[i - 1])
            p2 = np.array(lane_points[i])
            direction1 = p2 - p1

            p1 = np.array(lane_points[i])
            p2 = np.array(lane_points[i + 1])
            direction2 = p2 - p1
            direction = (direction1 + direction2) / 2.0

        rot = rotation_matrix(-np.pi / 2.0)
        orthonormal_vec = np.dot(rot, direction) / np.linalg.norm(direction)

        p = np.array(lane_points[i])
        right.append(p + orthonormal_vec * (width / 2.0))
        left.append(p - orthonormal_vec * (width / 2.0))

    if left_right:
        return left, right
    else:
        return geometry.Polygon(right + left[::-1])


def transform_polygon(polygon, ego_data, translate=False):
    x_center, y_center = ego_data[0], ego_data[1]
    yaw_rad, length, width = ego_data[2], ego_data[5], ego_data[6]

    polygon = affinity.translate(polygon, -x_center, -y_center)

    polygon = affinity.rotate(polygon, np.rad2deg(-yaw_rad + np.pi / 2), (0.0, 0.0))
    # polygon = affinity.rotate(
    #     polygon, -yaw_rad + np.pi / 2, origin="center", use_radians=True
    # )
    return polygon


def plot_polygon(img, size_meters, ppm, poly):
    x, y = poly.exterior.xy
    y = np.multiply(y, -1)

    x = np.add(x, size_meters / 2.0)
    y = np.add(y, size_meters / 2.0)

    x = np.multiply(x, ppm)
    y = np.multiply(y, ppm)

    xx, yy = polygon(x, y, shape=img.shape)
    img[yy, xx] = 255
    return img


def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.2989, 0.5870, 0.1140])
