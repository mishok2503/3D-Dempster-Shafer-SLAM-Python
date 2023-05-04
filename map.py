import numpy as np
from typing import Tuple

from lidar import LidarPointType
from robot import Robot
from cell import Cell
from util import bresenham3_line, numpy_map


class Map:
    def __init__(self, size: Tuple[int, int, int], cell_size: float, hole_size: int = 5):
        self.size = size
        self.cell_size = cell_size
        self.hole_size = hole_size
        rx, ry, rz = map(range, size)
        self.__map = np.array([[[Cell() for _ in rz] for _ in ry] for _ in rx])

    def get_center(self) -> np.array:
        return self.cell_size * numpy_map(lambda c: c / 2, self.size)

    def world2map(self, vec: np.array) -> Tuple[int, ...]:
        self.check_world_coords(vec)
        return tuple(map(lambda c: int(c / self.cell_size), vec))

    def map2world(self, coords: Tuple[int, int, int]) -> np.array:
        return numpy_map(lambda c: (c + 0.5) * self.cell_size, coords)

    def check_world_coords(self, vec: np.array):
        if (vec < 0).any():
            raise Exception("World coordinates less then zero")
        for i in [0, 1, 2]:
            if vec[i] >= self.size[i] * self.cell_size:
                raise Exception("World coordinates greater then map size")

    def get_score(self, robot: Robot, points: np.array) -> float:
        res = 0
        for lidar_point in points:
            if lidar_point.type == LidarPointType.UNKNOWN:
                continue
            point = lidar_point.point
            occupied = lidar_point.type == LidarPointType.POINT
            last = self.get_cell(self.world2map(robot.lidar2map(point)))
            res += last.p if occupied else 1 - last.p
        return res

    def update(self, robot: Robot, points: np.array):
        for lidar_point in points:
            if lidar_point.type == LidarPointType.UNKNOWN:
                continue
            point = lidar_point.point
            occupied = lidar_point.type == LidarPointType.POINT
            self.__beam_update(
                self.world2map(robot.position),
                self.world2map(robot.lidar2map(point)),
                occupied
            )

    def __beam_update(self, begin: Tuple[int, ...], end: Tuple[int, ...], occupied: bool):
        points = bresenham3_line(begin, end)
        quality = 0.7  # TODO
        for coords in points[:-self.hole_size]:
            self.__cell_update(coords, 0, quality)  # remove constant

        hole_coef = 0
        for coords in points[-self.hole_size:]:
            hole_coef += 1 / self.hole_size
            self.__cell_update(coords, occupied * hole_coef, quality)

    def __cell_update(self, coords: Tuple[int, ...], value: float, quality: float):
        self.get_cell(coords).update(value, quality)

    def get_cell(self, coords: Tuple[int, ...]) -> Cell:
        return self.__map[coords[0]][coords[1]][coords[2]]
