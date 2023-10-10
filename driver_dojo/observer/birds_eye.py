import numpy as np

from driver_dojo.observer import BaseObserver


class BirdsEyeObserver(BaseObserver):

    def __init__(self, config, vehicle, traffic_manager, renderer):
        super().__init__(config, vehicle, traffic_manager)
        self._renderer = renderer
        w, h = self.config.rendering.width, self.config.rendering.height
        self.low = np.zeros((w, h, 3), dtype=np.float64)
        self.high = np.ones((w, h, 3), dtype=np.float64) * 255

    def step(self):
        img = self._renderer.get_image()
        img = self._normalize_obs(img)  # Normalize
        return img

    def explain(self):
        return ['BirdsEye view']
