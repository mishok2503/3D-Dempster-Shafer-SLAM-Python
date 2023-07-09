import json

import numpy as np
from typing import Tuple

from lidar import LidarPointType
from robot import Robot
from cell import Cell, DSCell
from util import bresenham3_line, numpy_map


def dist(a, b):
    return max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2]))


class Map:
    def __init__(self, size: Tuple[int, int, int], cell_size: float, is_ds: bool = False, hole_size: int = 4):
        self.size = size
        self.cell_size = cell_size
        self.hole_size = hole_size
        self.__map = np.empty(size, dtype=object)
        self.__map.flat = [DSCell() for _ in self.__map.flat]
        self.__is_ds = is_ds

    def store(self, filename):
        with open(filename, "w") as file:
            t = [[[cell.get_p() > 0.5 for cell in row] for row in plane] for plane in self.__map]
            json.dump(t, file)

    def load(self, filename):
        with open(filename, "r") as file:
            data = json.load(file)
            self.__map = [[[DSCell(p) if self.__is_ds else Cell(p) for p in row] for row in plane] for plane in data]


    def get_center(self) -> np.array:
        return self.cell_size * numpy_map(lambda c: c / 2, self.size)

    def world2map(self, vec: np.array) -> Tuple[int, int, int]:
        # self.check_world_coords(vec)
        return int(vec[0] / self.cell_size), int(vec[1] / self.cell_size), int(vec[2] / self.cell_size)

    def map2world(self, coords: Tuple[int, int, int]) -> np.array:
        return (coords[0] + 0.5) * self.cell_size, (coords[1] + 0.5) * self.cell_size, (coords[2] + 0.5) * self.cell_size

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
        self.get_cell(coords).update(value)

    def get_cell(self, coords: Tuple[int, ...]) -> Cell:
        for i in [0, 1, 2]:
            if coords[i] < 0 or coords[i] >= self.size[i]:
                return Cell()
        return self.__map[coords[0]][coords[1]][coords[2]]

    def w2m(self, robot: Robot, vecs: np.array) -> np.array:
        return ((np.einsum("ij,kj->ik", vecs, robot.rotation_matrix) + robot.position) / self.cell_size).astype(int)

    def get_score(self, robot: Robot, points: np.array):
        t = np.array(self.world2map(robot.position))
        world_points = self.w2m(robot, np.array([p.point for p in points if p.type == LidarPointType.POINT]))
        return np.sum([self.get_cell(p).get_score() * np.linalg.norm(t - p) for p in world_points])

