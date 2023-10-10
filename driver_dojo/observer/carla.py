import numpy as np
from collections import deque
import time

from driver_dojo.core.types import CarlaSensor
from driver_dojo.core import CarlaObserverConfig
from driver_dojo.observer import BaseObserver


class CarlaCameraObserver(BaseObserver):
    def __init__(self, config, vehicle, traffic_manager, observer_config: CarlaObserverConfig):
        super().__init__(config, vehicle, traffic_manager)
        self.sensor_type = observer_config.sensor_type
        self.location = observer_config.location
        self.rotation = observer_config.rotation
        self.width = observer_config.width
        self.height = observer_config.height
        self.attributes = observer_config.attributes

        self._observer_config = observer_config
        self._queue = deque(maxlen=1)
        self.low = np.zeros(shape=(observer_config.width, observer_config.height, 3))
        if self.sensor_type == CarlaSensor.RGBCamera:
            self.high = np.ones_like(self.low) * 255
        elif self.sensor_type == CarlaSensor.DepthCamera:
            self.high = np.ones_like(self.low)

    def step(self):
        if len(self._queue) == 0:
            return np.zeros_like(self.low)  # TODO: Is this a good idea?
        img = self._queue.pop()
        img = self._normalize_obs(img)  # Normalize
        return img

    def reset(self):
        pass

    def listen(self, sensor_data):
        # See: https://github.com/carla-simulator/driving-benchmarks/blob/master/version084/carla/image_converter.py
        array = np.frombuffer(sensor_data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
        if self.sensor_type == CarlaSensor.RGBCamera:
            array = array[:, :, :3]
            array = array[:, :, ::-1]
        elif self.sensor_type == CarlaSensor.DepthCamera:
            array = array.astype(np.float32)
            array = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0])
            array /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)
            array = np.ones(array.shape) + (np.log(array) / 5.70378)
            array = np.clip(array, 0.0, 1.0)
            array *= 255.0
            array = np.repeat(array[:, :, np.newaxis], 3, axis=2)
        self._queue.append(array)

    def explain(self):
        return ['Camera']
