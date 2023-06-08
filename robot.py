import numpy as np
from util import rotation_matrix


class Robot:
    def __init__(self, pos: np.array = np.zeros(3), rot: np.array = np.zeros(3)):
        self.position = pos
        self.rotation = rot
        self.__rotation_matrix = rotation_matrix(rot)

    def lidar2map(self, point: np.array) -> np.array:
        return self.__rotation_matrix @ point + self.position

    def get_rotation_matrix(self):
        return self.__rotation_matrix

    def apply_true(self, delta_pos: np.array, delta_rot: np.array):
        self.rotation += delta_rot
        if (delta_rot != [0, 0, 0]).any():
            self.__rotation_matrix = rotation_matrix(self.rotation)
        self.position += self.__rotation_matrix @ delta_pos

    def apply_odometry(self, delta_pos: np.array, delta_rot: np.array, world, data, samples: int = 2000):
        best, score = None, -1
        probit = 1
        stddev_pos = np.linalg.norm(delta_pos) / probit
        stddev_rot = 0.3 / probit

        res = []

        for i in range(samples):
            rot = self.rotation + delta_rot + np.random.normal(0, stddev_rot, 3)
            pos = self.position + rotation_matrix(rot) @ delta_pos + np.random.normal(0, stddev_pos, 3)
            new_robot = Robot(pos, rot)
            s = world.get_score(new_robot, data)
            res.append(new_robot)
            if s > score:
                score = s
                best = new_robot

        self.copy(best)
        return res

    def copy(self, other: 'Robot'):
        self.position = other.position
        self.rotation = other.rotation
        self.__rotation_matrix = rotation_matrix(other.rotation)
