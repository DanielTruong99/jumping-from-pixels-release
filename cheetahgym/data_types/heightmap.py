import numpy as np


class Heightmap:
    def __init__(self):
        self.filename = None
        self.scale = 1
        self.shift = 0
        self.save_dir = "./terrains/default/test"
        self.resolution = 0.02
        self.length_px = 1000
        self.width_px = 100

        self.gap_params = {}

    def load_from_file(self):
        raise NotImplementedError
