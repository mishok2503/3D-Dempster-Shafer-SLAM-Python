import numpy as np
from typing import Tuple

from lidar import LidarPointType
from robot import Robot
from cell import Cell
from util import bresenham3_line, numpy_map


class Map:
    def __init__(self, size: Tuple[int, int, int], cell_size: float, hole_size: int = 4):
        self.size = size
        self.cell_size = cell_size
        self.hole_size = hole_size
        rx, ry, rz = map(range, size)
        self.__map = np.array([[[Cell() for _ in rz] for _ in ry] for _ in rx])

    def get_center(self) -> np.array:
        return self.cell_size * numpy_map(lambda c: c / 2, self.size)

    def world2map(self, vec: np.array) -> Tuple[int, int, int]:
        # self.check_world_coords(vec)
        return int(vec[0] / self.cell_size), int(vec[1] / self.cell_size), int(vec[2] / self.cell_size)

    def map2world(self, coords: Tuple[int, int, int]) -> np.array:
        return (coords[0] + 0.5) * self.cell_size, (coords[1] + 0.5) * self.cell_size, (coords[2] + 0.5) * self.cell_size

    def update(self, robot: Robot, points: np.array):
        for lidar_point in points:
            if lidar_point.type == LidarPointType.UNKNOWN:
                continue
            point = lidar_point.point
            occupied = lidar_point.type == LidarPointType.POINT
            try:
                self.__beam_update(
                    self.world2map(robot.position),
                    self.world2map(robot.lidar2map(point)),
                    occupied
                )
            except Exception:
                pass

    def __beam_update(self, begin: Tuple[int, ...], end: Tuple[int, ...], occupied: bool):
        points = bresenham3_line(begin, end)
        quality = 0.7  # TODO
        for coords in points[:-self.hole_size]:
            self.__cell_update(coords, 0, quality)  # remove constant

        hole_coef = 0
        for coords in points[-self.hole_size:]:
            hole_coef += 1 / self.hole_size
            self.__cell_update(coords, (1 + occupied * hole_coef) / 2, quality)

    def __cell_update(self, coords: Tuple[int, ...], value: float, quality: float):
        self.get_cell(coords).update(value, quality)

    def get_cell(self, coords: Tuple[int, ...]) -> Cell:
        return self.__map[coords[0]][coords[1]][coords[2]]

    def w2m(self, robot: Robot, vecs: np.array) -> np.array:
        return ((np.einsum("ij,kj->ik", vecs, robot.rotation_matrix) + robot.position) / self.cell_size).astype(int)

    def get_score(self, robot: Robot, points: np.array):
        world_points = self.w2m(robot, np.array([p.point for p in points if p.type == LidarPointType.POINT]))
        return np.sum([self.get_cell(p).get_p() for p in world_points])

