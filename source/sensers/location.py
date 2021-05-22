class Location:
    """
    Класс хранящий в себе местопложение средний точки объекта,
    его ширину, высоту и длинну, и классификацию объекта
    """
    def __init__(self, *, label=None, pos, weight: float, height: float, lenght: float, points=None):
        self.label = label
        self.pos = pos
        self.weight = weight
        self.height = height
        self.lenght = lenght
        self.points = points