import numpy as np
from util import rotation_matrix


class Robot:
    def __init__(self, pos: np.array = np.zeros(3), rot: np.array = np.zeros(3)):
        self.position = pos
        self.rotation = rot
        self.rotation_matrix = rotation_matrix(rot)

    def lidar2map(self, point: np.array) -> np.array:
        return self.rotation_matrix @ point + self.position

    def apply_true(self, delta_pos: np.array, delta_rot: np.array):
        self.rotation += delta_rot
        self.position += rotation_matrix(self.rotation) @ delta_pos

    def apply_odometry(self, delta_pos: np.array, delta_rot: np.array, world, data, samples: int = 300):
        best, score = None, -1
        probit = 50
        stddev_pos = 1 / probit
        stddev_rot = 1 / probit
        for i in range(samples):
            delta_rot = [0, 0, np.random.normal(0, stddev_rot)]
            delta_pos = np.random.normal(0, stddev_pos, 3)
            delta_pos[2] = 0
            rot = self.rotation + delta_rot
            pos = self.position + rotation_matrix(rot) @ delta_pos
            new_robot = Robot(pos, rot)
            s = world.get_score(new_robot, data)
            if s > score:
                score = s
                best = new_robot

        if score != -1:
            self.copy(best)
            print("GOOD")
        else:
            print("BAD")

    def copy(self, other: 'Robot'):
        self.position = other.position
        self.rotation = other.rotation
        self.rotation_matrix = other.rotation_matrix
