from math import *
from typing import Tuple, List
import numpy as np


def numpy_map(f, v) -> np.array:
    return np.array(list(map(f, v)))


def rotation_matrix(rotation: np.array) -> np.array:
    a, b, c = rotation
    return np.linalg.inv(np.array([
        [cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c), sin(a) * sin(b)],
        [sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b)],
        [sin(b) * sin(c), sin(b) * cos(c), cos(b)]
    ]))


# https://www.geeksforgeeks.org/bresenhams-algorithm-for-3-d-line-drawing
def bresenham3_line(begin: Tuple[int, ...], end: Tuple[int, ...], more_steps: int = 0) -> List[Tuple[int, ...]]:
    x1, y1, z1 = begin
    x2, y2, z2 = end
    results = [(x1, y1, z1)]
    dx, dy, dz = abs(x2 - x1), abs(y2 - y1), abs(z2 - z1)
    xs, ys, zs = map(lambda v: 1 if v[1] > v[0] else -1, zip(begin, end))

    # Driving axis is X-axis
    if dx >= dy and dx >= dz:
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while x1 != x2 + xs * more_steps:
            x1 += xs
            if p1 >= 0:
                y1 += ys
                p1 -= 2 * dx
            if p2 >= 0:
                z1 += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            results.append((x1, y1, z1))

    # Driving axis is Y-axis
    elif dy >= dx and dy >= dz:
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while y1 != y2 + ys * more_steps:
            y1 += ys
            if p1 >= 0:
                x1 += xs
                p1 -= 2 * dy
            if p2 >= 0:
                z1 += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            results.append((x1, y1, z1))

    # Driving axis is Z-axis
    else:
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while z1 != z2 + zs * more_steps:
            z1 += zs
            if p1 >= 0:
                y1 += ys
                p1 -= 2 * dz
            if p2 >= 0:
                x1 += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            results.append((x1, y1, z1))
    return results
