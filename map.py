import numpy as np
from typing import Tuple

from lidar import LidarPointType
from robot import Robot
from cell import Cell, DSCell
from util import bresenham3_line, numpy_map


class Map:
    def __init__(self, size: Tuple[int, int, int], cell_size: float, is_ds: bool = False, hole_size: int = 4):
        self.size = size
        self.cell_size = cell_size
        self.hole_size = hole_size
        rx, ry, rz = map(range, size)
        self.__map = np.array([[[DSCell() if is_ds else Cell() for _ in rz] for _ in ry] for _ in rx])

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
        points = bresenham3_line(begin, end, self.hole_size)
        full_hole = self.hole_size * 2 + 1
        if len(points) < full_hole + 1:
            return
        for coords in points[:-full_hole]:
            self.__cell_update(coords, 0)

        for i in range(full_hole):
            self.__cell_update(points[i - full_hole], 1 - abs(i - self.hole_size) / self.hole_size)

    def __cell_update(self, coords: Tuple[int, ...], value: float):
        self.get_cell(coords).update(value)

    def get_cell(self, coords: Tuple[int, ...]) -> Cell:
        return self.__map[coords[0]][coords[1]][coords[2]]

    def w2m(self, robot: Robot, vecs: np.array) -> np.array:
        return ((np.einsum("ij,kj->ik", vecs, robot.rotation_matrix) + robot.position) / self.cell_size).astype(int)

    def get_score(self, robot: Robot, points: np.array):
        world_points = self.w2m(robot, np.array([p.point for p in points if p.type == LidarPointType.POINT]))
        return np.sum([self.get_cell(p).get_score() for p in world_points])

