class Cell:
    def __init__(self, p: float = 0.5):
        self.__sum = p
        self.__n = 1

    # def update(self, value: float, quality: float):
    #     self.p = (1.0 - quality) * self.p + quality * value
    #     return self.p > 0.6
    def update(self, value: float, quality: float):
        self.__sum += value
        self.__n += 1

    def get_p(self):
        return self.__sum / self.__n
