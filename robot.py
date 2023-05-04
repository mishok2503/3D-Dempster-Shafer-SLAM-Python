import numpy as np
from util import rotation_matrix


class Robot:
    def __init__(self, pos: np.array = np.zeros(3), rot: np.array = np.zeros(3)):
        self.position = pos
        self.rotation = rot
        self.__rotation_matrix = rotation_matrix(rot)

    def lidar2map(self, point: np.array) -> np.array:
        return self.__rotation_matrix @ point + self.position

    def apply_true(self, delta_pos: np.array, delta_rot: np.array):
        self.rotation += delta_rot
        self.position += rotation_matrix(self.rotation) @ delta_pos

    def apply_odometry(self, delta_pos: np.array, delta_rot: np.array, world, data, samples: int = 150):
        best, score = None, -1
        probit = 3
        stddev_pos = np.linalg.norm(delta_pos) / probit
        stddev_rot = 0.03 / probit
        for i in range(samples):
            delta_rot += np.random.normal(0, stddev_rot, 3)
            delta_pos += np.random.normal(0, stddev_pos, 3)
            rot = self.rotation + delta_rot
            pos = self.position + rotation_matrix(rot) @ delta_pos
            new_robot = Robot(pos, rot)
            s = world.get_score(new_robot, data)
            if s > score:
                score = s
                best = new_robot

        self.copy(best)

    def copy(self, other: 'Robot'):
        self.position = other.position
        self.rotation = other.rotation
        self.__rotation_matrix = other.__rotation_matrix
