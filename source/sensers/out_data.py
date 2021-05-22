import os
from .location import Location


class OutData:
    """
    Класс хранящий в себе путь до изображения и сопутствующею
    ей метаданные
    """
    def __init__(self, f_name: str, ground_loc: Location, door_loc:Location, objs_loc):
        assert os.path.isfile(f_name)
        self.f_name = f_name
        self.ground_loc = ground_loc
        self.door_loc = door_loc
        self.objs_loc = objs_loc

    def save_file(self, f_path):
        """
        Сохранить данный объект со всеми своими данными по данному пути
        """
        pass


    def prepare4json(self):
        """
        Возвращает словарь данного объекта, для сохранения в json
        """
        pass

