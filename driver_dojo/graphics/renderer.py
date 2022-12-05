import numpy as np
from typing import List
import pyglet
from pyglet.window import key

from .camera import CenteredCamera
from .sprites.car import CarSprite
from .sprites.street import LaneSprite, NodeSprite
from ..vehicle.attachments import SubGoalAttachment, WaypointAttachment

NON_EGO_COLOR = (255, 0, 0)
EGO_COLOR = (255, 255, 0)
SUB_GOAL_COLOR = (0, 0, 255)
WAYPOINT_COLOR = (0, 255, 0)
SUB_GOAL_SIZE = 1.6
WAYPOINT_SIZE = 0.4


class Renderer:
    def __init__(self, config, traffic_manager, street_map):
        self.config = config
        self.traffic_manager = traffic_manager
        self.street_map = street_map
        self._width = config.rendering.width
        self._height = config.rendering.height
        self._visible = config.rendering.human_mode

        self._window: pyglet.window.Window = pyglet.window.Window(self._width, self._height)  # This one is for ego centered view
        self._keys = key.KeyStateHandler()
        self._window.push_handlers(self._keys)
        pyglet.gl.glClearColor(1, 1, 1, 1)  # White background
        self._window.set_visible(self._visible)
        self._camera = CenteredCamera(self._window)
        self._camera.zoom = min(self._width, self._height) // (2 * self.config.rendering.radius)

        self._window_global: pyglet.window.Window = pyglet.window.Window(self._width, self._height)  # This one is to render the map layout
        self._window_global.switch_to()
        self._window_global.set_visible(False)
        self._camera_global = CenteredCamera(self._window_global)
        self._window.switch_to()

        self._road_batch: pyglet.graphics.Batch = pyglet.graphics.Batch()
        self._dyn_batch: pyglet.graphics.Batch = pyglet.graphics.Batch()
        self._back_group = pyglet.graphics.OrderedGroup(0)
        self._mid_group = pyglet.graphics.OrderedGroup(1)
        self._fore_group = pyglet.graphics.OrderedGroup(2)

        self._cars: List[CarSprite] = []
        self._car_ids: List[str] = []
        self._lanes: List[LaneSprite] = []
        self._nodes: List[NodeSprite] = []
        self._waypoints: List = []
        self._traci = None
        self._vehicle = None
        self._scenario = None
        self._grabbed_image = False

    def reset(self, traci, vehicle, scenario):
        self._traci = traci
        self._vehicle = vehicle
        self._scenario = scenario

        # Setup global camera
        self._window_global.switch_to()
        x_min, y_min, x_max, y_max = self._scenario.sumo_net.getBoundary()
        x_center, y_center = (x_max - x_min) / 2, (y_max - y_min) / 2
        self._camera_global.position = (x_center, y_center)
        max_side = max(x_max - x_min, y_max - y_min)
        self._camera_global.zoom = 1 / (max_side / min(self._height, self._width))
        self._window.switch_to()

        [x.delete() for x in self._cars + self._lanes + self._nodes]
        self._window.clear()
        self._window_global.clear()
        self._keys.clear()

        self._cars = []
        self._car_ids = []
        self._lanes = []
        self._nodes = []
        self.add_lanes()
        self.add_nodes()

        pyglet.clock.tick()
        for i in range(2):
            self._draw()
            # self._window_global.switch_to()
            self._draw_map()
            # self._window.switch_to()

    def close(self):
        self._window.close()
        self._window_global.close()

    def step(self):
        removed_car_ids = [x for x in self._car_ids if x not in self.traffic_manager.actor_ids]
        for x in removed_car_ids:
            self.remove_car(x)
        new_car_ids = [x for x in self.traffic_manager.actor_ids if x not in self._car_ids]  # Add new cars
        for x in new_car_ids:
            self.add_car(x)

        ego_id = self.config.simulation.egoID
        ego_state = self.traffic_manager.get_actor_state(ego_id)

        [car.update() for car in self._cars]

        wps_drawn = []  # Draw waypoints and sub-goals
        if self._vehicle.has_attachment(WaypointAttachment):
            # wps = self._vehicle.waypoints  # Waypoints
            # wps_drawn.extend(self._draw_waypoints(wps, WAYPOINT_COLOR, WAYPOINT_SIZE))

            # Cubic course over waypoints
            xs, ys, _, _, _ = self._vehicle.get_attachment(WaypointAttachment).cubic_spline_course(self._vehicle.location[:2], res=self.config.vehicle.course_resolution)
            from driver_dojo.common.waypoint import SimpleWaypoint
            wps = [SimpleWaypoint((x, y)) for x, y in zip(xs, ys)]
            wps_drawn.extend(self._draw_waypoints(wps, WAYPOINT_COLOR, WAYPOINT_SIZE))

        if self._vehicle.has_attachment(SubGoalAttachment):
            wps = self._vehicle.sub_goals
            wps_drawn.extend(self._draw_waypoints(wps, SUB_GOAL_COLOR, SUB_GOAL_SIZE))

        self._camera.position = ego_state.location[:2]
        self._camera.rotation = -np.degrees(ego_state.rotation[1])

        self._draw()
        return self.get_image()

    @property
    def visible(self):
        return self._visible

    @visible.setter
    def visible(self, x: bool):
        self._visible = x
        self._window.set_visible(x)

    def get_image(self, global_view=False):
        if global_view:
            self._window_global.switch_to()
            self._draw_map()
            self._draw_map()
            self._draw_map()
        else:
            self._window.switch_to()

        buffer = pyglet.image.get_buffer_manager().get_color_buffer()
        image_data = buffer.get_image_data()
        arr = np.frombuffer(image_data.get_data(), dtype=np.uint8)
        arr = arr.reshape(buffer.height, buffer.width, 4)
        arr = arr.astype(np.float32) / 255.
        arr = arr[::-1, :, 0:3]
        arr = np.rollaxis(arr, 2)

        if global_view:
            self._window.switch_to()
        return arr

    def add_car(self, actor_id):
        color = NON_EGO_COLOR if actor_id != self.config.simulation.egoID else EGO_COLOR
        self._cars.append(
            CarSprite(actor_id, self.traffic_manager, color, self._dyn_batch, self._fore_group)
        )
        self._car_ids.append(actor_id)

    def remove_car(self, actor_id):
        if actor_id in self._car_ids:
            index = self._car_ids.index(actor_id)
            self._cars[index].delete()
            del self._cars[index]
            del self._car_ids[index]

    def add_lanes(self):
        for lane_id, lane_node in self.street_map.graph.lanes.items():
            if lane_node.function == 'internal':  # Internal lane
                continue
            self._lanes.append(
                LaneSprite(self.street_map, lane_id, self._road_batch, self._mid_group, self._back_group)
            )

    def add_nodes(self):
        for node_id, node in self.street_map.graph.junctions.items():
            self._nodes.append(
                NodeSprite(self.street_map, node_id, self._road_batch, self._mid_group, self._back_group)
            )

    def _draw_waypoints(self, waypoints, color, size):
        circles = []
        for sub_goal in waypoints:
            x, y = sub_goal.location
            circles.append(pyglet.shapes.Circle(x, y, size, color=color, batch=self._dyn_batch, group=self._fore_group))
        return circles

    @property
    def keyboard_state(self):
        return self._keys

    def _draw(self):
        @self._window.event
        def on_draw():
            self._window.clear()
            with self._camera:
                self._road_batch.draw()
                self._dyn_batch.draw()

        pyglet.clock.tick()
        self._window.dispatch_events()
        self._window.dispatch_event('on_draw')
        self._window.flip()

    def _draw_map(self):
        @self._window_global.event
        def on_draw():
            self._window_global.clear()
            with self._camera_global:
                self._road_batch.draw()  # only draw the road

        self._window_global.dispatch_events()
        self._window_global.dispatch_event('on_draw')
        self._window_global.flip()
