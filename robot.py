import numpy as np
from util import rotation_matrix

class Robot:
    def __init__(self, pos: np.array = np.zeros(3), rot: np.array = np.zeros(3)):
        self.position = pos
        self.rotation = rot
        self.__rotation_matrix = rotation_matrix(rot)

    def lidar2map(self, point: np.array) -> np.array:
        return self.__rotation_matrix @ point + self.position

    def apply_odometry(self, delta_pos: np.array, delta_rot: np.array):
        if not np.allclose(delta_rot, [0, 0, 0]):
            self.rotation += delta_rot
            self.__rotation_matrix = rotation_matrix(self.rotation)
        self.position += self.__rotation_matrix @ delta_pos
