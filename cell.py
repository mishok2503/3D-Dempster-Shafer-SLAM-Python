class Cell:
    def __init__(self, p: float = 0.5):
        self.p = p

    def update(self, value: float, quality: float):
        self.p = (1.0 - quality) * self.p + quality * value
        return self.p > 0.6
