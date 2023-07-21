import numpy as np
from util import rotation_matrix


class Robot:
    def __init__(self, pos: np.array = np.zeros(3), rot: np.array = np.zeros(3)):
        self.position = pos
        self.rotation = rot
        self.rotation_matrix = rotation_matrix(rot)

    def lidar2world(self, point: np.array) -> np.array:
        return self.rotation_matrix @ point + self.position

    def apply_odometry(self, delta_pos: np.array, delta_rot: np.array, world, data, samples: int = 350):
        stddev_pos = 0.3
        stddev_rot = 0.4

        self.rotation += delta_rot
        if (delta_rot != [0, 0, 0]).any():
            self.rotation_matrix = rotation_matrix(self.rotation)
        self.position += self.rotation_matrix @ delta_pos

        if samples == 0:
            return

        best, score = self, world.get_score(self, data)

        for i in range(samples):
            dr = [np.random.normal(0, stddev_rot), 0, 0]
            # dr = np.random.normal(0, stddev_rot, 3)
            dp = [np.random.normal(0, stddev_pos), np.random.normal(0, stddev_pos), 0]
            # dp = np.random.normal(0, stddev_rot, 3)
            rot = self.rotation + dr
            pos = self.position + dp
            new_robot = Robot(pos, rot)
            s = world.get_score(new_robot, data)
            if s > score:
                score = s
                best = new_robot

        self.copy(best)

    def copy(self, other: 'Robot'):
        self.position = other.position
        self.rotation = other.rotation
        self.rotation_matrix = rotation_matrix(other.rotation)
