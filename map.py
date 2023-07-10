import json

import numpy as np
from typing import Tuple

from lidar import LidarPointType
from robot import Robot
from util import bresenham3_line, numpy_map


def get_score(cell: np.array):
    return cell[1] + cell[2] / 2


def dist(a, b):
    return max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2]))


def update(cell: np.array, value: float):
    t = [0.95 - value, value - 0.05, 0.1]
    cell[0] = cell[0] * t[0] + cell[0] * t[2] + cell[2] * t[0]
    cell[1] = cell[1] * t[1] + cell[1] * t[2] + cell[2] * t[1]
    cell[2] = cell[2] * t[2]
    tt = cell[0] + cell[1] + cell[2]
    cell[0] /= tt
    cell[1] /= tt
    cell[2] /= tt


class Map:
    def __init__(self, size: Tuple[int, int, int], cell_size: float, is_ds: bool = False, hole_size: int = 4):
        self.size = size
        self.cell_size = cell_size
        self.hole_size = hole_size
        self.__map = np.empty((size[0], size[1], size[2], 3), dtype=float)
        for plane in self.__map:
            for row in plane:
                for cell in row:
                    cell[0] = 0.45
                    cell[1] = 0.45
                    cell[2] = 0.1
        self.__is_ds = is_ds

    def get_center(self) -> np.array:
        return self.cell_size * numpy_map(lambda c: c / 2, self.size)

    def world2map(self, vec: np.array) -> Tuple[int, int, int]:
        # self.check_world_coords(vec)
        return int(vec[0] / self.cell_size), int(vec[1] / self.cell_size), int(vec[2] / self.cell_size)

    def map2world(self, coords: Tuple[int, int, int]) -> np.array:
        return (coords[0] + 0.5) * self.cell_size, (coords[1] + 0.5) * self.cell_size, (
                    coords[2] + 0.5) * self.cell_size

    def update(self, robot: Robot, points: np.array):
        for lidar_point in points:
            if lidar_point.type == LidarPointType.POINT:
                self.__beam_update(
                    self.world2map(robot.position),
                    self.world2map(robot.lidar2world(lidar_point.point)),
                )

    def __beam_update(self, begin: Tuple[int, ...], end: Tuple[int, ...]):
        points = bresenham3_line(begin, end, 0)
        for coords in points:
            if dist(coords, end) < self.hole_size:
                break
            self.__cell_update(coords, 0)

        for i in range(-self.hole_size, self.hole_size + 1):
            for j in range(-self.hole_size, self.hole_size + 1):
                for k in range(-self.hole_size, self.hole_size + 1):
                    coords = (end[0] + i, end[1] + j, end[2] + k)
                    self.__cell_update(coords, 0.5 + 0.5 / (1 + dist(end, coords)))

    def __cell_update(self, coords: Tuple[int, ...], value: float):
        update(self.get_cell(coords), value)

    def get_cell(self, coords: Tuple[int, ...]) -> np.array:
        for i in [0, 1, 2]:
            if coords[i] < 0 or coords[i] >= self.size[i]:
                return np.array([0.45, 0.45, 0.1])
        return self.__map[coords[0]][coords[1]][coords[2]]

    def w2m(self, robot: Robot, vecs: np.array) -> np.array:
        return ((np.einsum("ij,kj->ik", vecs, robot.rotation_matrix) + robot.position) / self.cell_size).astype(int)

    def get_score(self, robot: Robot, points: np.array):
        t = np.array(self.world2map(robot.position))
        world_points = self.w2m(robot, np.array([p.point for p in points if p.type == LidarPointType.POINT]))
        return np.sum([get_score(self.get_cell(p)) * np.linalg.norm(t - p) for p in world_points])
