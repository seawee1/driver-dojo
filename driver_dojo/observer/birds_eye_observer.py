import matplotlib.pyplot as plt
import numpy as np

from driver_dojo.observer import BaseObserver
from driver_dojo.common.utils import (
    net_to_polygons,
    car_to_polygon,
    transform_polygon,
    plot_polygon,
    rgb2gray,
    create_observation_space,
)
import driver_dojo.common.runtime_vars as runtime_vars
from driver_dojo.common.utils import normalize_observations, create_observation_space
import matplotlib.pyplot as plt


class BirdsEyeObserver(BaseObserver):
    def __init__(self):
        super().__init__()
        self.lane_polygons = None
        self.size_meters = runtime_vars.config["observations"]["beo_size_meters"]
        self.ppm = runtime_vars.config["observations"]["beo_ppm"]
        self.size = int(self.size_meters * self.ppm)
        self.num_channels = 3

        self.low = np.zeros(shape=(self.size, self.size, self.num_channels))
        self.high = np.ones_like(self.low) * 255

        # For now, we don't support image normalization. This is due to SB3 CnnPolicy implementation.
        self.observation_space = create_observation_space(
            self.low,
            self.high,
            runtime_vars.config["observations"]["feature_scaling"],
        )

        self.draw_subgoals = runtime_vars.config.observations.beo_draw_subgoals

        # Create figure and set to exact pixel size
        plt.rcParams["figure.dpi"] = 100
        self.fig = plt.figure(
            figsize=(
                self.size / 100,
                self.size / 100,
            ),  # (self.size / dpi, self.size / dpi),
            # dpi=dpi,
            frameon=False,
            facecolor="black",
        )

    def _interesting_traffic(self):
        dists = runtime_vars.traffic_manager.distance_to_ego
        if dists is None:
            return []

        traffic_data = runtime_vars.traffic_manager.traffic_state_transformed()
        # mask = np.argwhere(dists <= self.size_meters / 2.0).flatten()

        return traffic_data  # [mask]

    def observe(self):

        ego_data = runtime_vars.traffic_manager.ego_state_transformed()
        traffic_data = self._interesting_traffic()

        self.ax = plt

        self.fig.patch.set_facecolor("black")
        self.ax = self.fig.add_subplot(111)
        self.ax.patch.set_facecolor("black")
        self.ax.set(facecolor="black")

        def draw():
            self.ax.axis("off")
            self.ax.spines["top"].set_visible(False)
            self.ax.spines["right"].set_visible(False)
            self.ax.spines["bottom"].set_visible(False)
            self.ax.spines["left"].set_visible(False)
            self.ax.set_xlim([int(-self.size_meters), int(self.size_meters)])
            self.ax.set_ylim([int(-self.size_meters), int(self.size_meters)])
            plt.gca().set_position([0, 0, 1, 1])
            plt.box(False)
            self.fig.canvas.draw()

        def plt_to_numpy(fig):
            img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep="")
            img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            self.fig.clear()
            return img

        # Draw lanes
        lane_polys = []
        for lane_poly in self.lane_polygons:
            lane_poly = transform_polygon(lane_poly, ego_data, translate=True)
            xs, ys = zip(*lane_poly.exterior.coords)
            lane_polys.extend([xs, ys, "gray"])
        self.ax.fill(*lane_polys)

        # Draw ego
        ego_poly = car_to_polygon(ego_data)
        ego_poly = transform_polygon(ego_poly, ego_data, translate=True)
        xs, ys = zip(*ego_poly.exterior.coords)
        self.ax.fill(xs, ys, "green")

        # Draw traffic
        traffic_polys = []
        for car_data in traffic_data:
            car_poly = car_to_polygon(car_data)
            car_poly = transform_polygon(car_poly, ego_data, translate=True)
            xs, ys = zip(*car_poly.exterior.coords)
            traffic_polys.extend([xs, ys, "red"])
        self.ax.fill(*traffic_polys)

        # Draw subgoals
        if self.draw_subgoals:
            goals = runtime_vars.vehicle.sub_goals
            if goals and len(goals) > 0:
                from shapely.geometry import Point

                # x_ego, y_ego = ego_data[0], ego_data[1]
                for goal in goals:
                    x, y = goal.position
                    point = Point(x, y)
                    point = transform_polygon(point, ego_data, translate=True)
                    circle = plt.Circle(point.xy, 1.0, color="yellow")
                    self.ax.add_patch(circle)

        # Draw and to numpy
        draw()
        img = plt_to_numpy(self.fig)

        # Debug
        if (
            runtime_vars.config.debug.plot_birds_eye_observations
            and runtime_vars.time_step
            % runtime_vars.config.debug.plot_birds_eye_observations
            == 0
        ):
            from PIL import Image

            im = Image.fromarray(img)
            im.save("beo_debug.png")

        img = normalize_observations(
            img,
            self.low,
            self.high,
            runtime_vars.config["observations"]["feature_scaling"],
        )

        return img

    def reset(self):
        self.lane_polygons = net_to_polygons(runtime_vars.net)
