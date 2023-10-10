import pyglet
import numpy as np


class CarSprite(pyglet.shapes.Rectangle):
    def __init__(self, actor_id, traffic_manager, color, batch=None, group=None):
        super().__init__(0, 0, 1, 1, batch=batch, group=group)
        self.traffic_manager = traffic_manager
        self._actor_id = actor_id
        self._color = color

    def update(self):
        actor_state = self.traffic_manager.get_actor_state(self._actor_id)
        # self.position = world.world.traci.vehicle.getPosition(self._actor_id)
        self.position = actor_state.location[:2]
        self.width = actor_state.width
        self.height = actor_state.length
        self.anchor_position = self.width / 2.0, self.height / 2.0
        self.rotation = -np.degrees(actor_state.rotation[1]) + 90.0  # np.degrees(actor_state.rotation[1])
        self.color = self._color
        self.opacity = 255
