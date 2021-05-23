class Geometry:
    """
    Класс хранящий в себе местопложение средний точки объекта,
    его ширину, высоту и длинну, и классификацию объекта
    """
    def __init__(self, posotion=None, rotation=None, dimensions=None):
        self.pos = posotion
        self.rot = rotation
        self.dim = dimensions

    def to_object(self) -> dict:
        """
        Функция возвращающая объект геометрии как словарь
        """
        if self.pos == None and self.rot == None and self.dim == None:
            return 'unknown'
        res_dict = {}
        if self.pos != None:
            assert 'x' in self.pos
            assert isinstance(self.pos['x'], float)
            assert 'y' in self.pos
            assert isinstance(self.pos['y'], float)
            assert 'z' in self.pos
            assert isinstance(self.pos['z'], float)
            res_dict['position'] = {
                'x': self.pos['x'],
                'y': self.pos['y'],
                'z': self.pos['z'],
            }
        else:
            res_dict['position'] = 'unknown'

        if self.rot != None:
            assert 'x' in self.rot
            assert isinstance(self.rot['x'], float)
            assert 'y' in self.rot
            assert isinstance(self.rot['y'], float)
            assert 'z' in self.rot
            assert isinstance(self.rot['z'], float)
            res_dict['rotation'] = {
                'x': self.rot['x'],
                'y': self.rot['y'],
                'z': self.rot['z'],
            }
        else:
            res_dict['rotation'] = 'unknown'

        if self.dim != None:
            assert 'x' in self.dim
            assert isinstance(self.dim['x'], float)
            assert 'y' in self.dim
            assert isinstance(self.dim['y'], float)
            assert 'z' in self.dim
            assert isinstance(self.dim['z'], float)
            res_dict['dimensions'] = {
                'x': self.dim['x'],
                'y': self.dim['y'],
                'z': self.dim['z'],
            }
        else:
            res_dict['dimensions'] = 'unknown'

        return res_dict


__all__ = ['Geometry']
