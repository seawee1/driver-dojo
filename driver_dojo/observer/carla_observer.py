import numpy as np

from driver_dojo.common import utils
from driver_dojo.observer import BaseObserver
from driver_dojo.common.utils import normalize_observations, create_observation_space
import driver_dojo.common.runtime_vars as runtime_vars

from collections import deque


class CarlaCameraObserver(BaseObserver):
    def __init__(self):
        super().__init__()
        self.low = np.zeros(shape=(84, 84, 3))
        self.high = np.ones_like(self.low) * 255
        self.cnt = 0

        # For now, we don't support image normalization. This is due to SB3 CnnPolicy implementation.
        self.observation_space = create_observation_space(
            self.low,
            self.high,
            runtime_vars.config["observations"]["feature_scaling"],
        )

        self.rgb_queue = deque(maxlen=1)

    def observe(self):
        import time
        while len(self.rgb_queue) == 0:
            time.sleep(0.01)

        image = self.rgb_queue.pop()
        image = normalize_observations(
            image,
            self.low,
            self.high,
            runtime_vars.config["observations"]["feature_scaling"],
        )
        return image

    def listen_rgb(self, sensor_data):
        sensor_data.save_to_disk(f'carla_rgb_{self.cnt}.png')
        self.cnt += 1
        array = np.frombuffer(sensor_data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.rgb_queue.append(array)
