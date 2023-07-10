import json


def intersection(a, b):
    return a[0] & b[0], a[1] & b[1]


def combine(a, b):
    c = {(0, 0): 0, (1, 0): 0, (0, 1): 0, (1, 1): 0}
    for k1, v1 in a.items():
        for k2, v2 in b.items():
            k = intersection(k1, k2)
            c[k] += v1 * v2
    # t = 1 / (1 - c[(0, 0)])
    tt = c[(1, 0)] + c[(0, 1)] + c[(1, 1)]
    for i in c:
        c[i] /= tt
    c[(0, 0)] = 0
    return c


class DSCell:
    def __init__(self, p: float = 0.5):
        self.p = DSCell.get_ds(p)

    def update(self, value: float):
        self.p = combine(self.p, DSCell.get_ds(value))
        res = 0
        for i in self.p:
            res += self.p[i]
        if abs(1 - res) > 0.1:
            print(self.p)
            input()

    @staticmethod
    def get_ds(p: float):
        return {
            (0, 0): 0,
            (1, 0): 1 - p - 0.05,
            (0, 1): p - 0.05,
            (1, 1): 0.1
        }

    def get_p(self):
        return self.p[(0, 1)] + self.p[(1, 1)] / 2

    def get_score(self) -> float:
        return self.get_p()


class Cell:
    def __init__(self, p: float = 0.5):
        self.p = p

    def update(self, value: float):
        self.p = (self.p + value) / 2

    def get_p(self):
        return self.p

    def get_score(self) -> float:
        return self.get_p()

# class Cell:
#     def __init__(self, p: float = 0.5):
#         self.sum = p
#         self.n = 1
#
#     def update(self, value: float):
#         self.sum += value
#         self.n += 1
#
#     def get_p(self):
#         return self.sum / self.n
#
#     def get_score(self) -> float:
#         return self.get_p()
