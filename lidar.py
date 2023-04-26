import numpy as np


class LidarPointType:
    UNKNOWN = 0
    POINT = 1
    MAX = 2


class LidarPoint:
    def __init__(self, point: np.array, point_type: LidarPointType):
        self.point = point
        self.type = point_type
