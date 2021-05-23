import os
from source.sensers.figure import Figure
from source.sensers.geometry import Geometry


class OutData:
    """
    Класс хранящий в себе путь до изображения и сопутствующею
    ей метаданные
    """
    def __init__(self, f_name: str):
        assert os.path.isfile(f_name)
        self.f_name = f_name
        self.figures = []

    def add_figure(self, figure: Figure) -> None:
        """
        Добавление фигуры к данному объекту
        """
        self.figures.append(figure)

    def save_file(self, f_path):
        """
        Сохранить данный объект со всеми своими данными по данному пути
        """
        pass


    def to_object(self):
        """
        Возвращает словарь данного объекта, для сохранения в json
        """
        res_dict = {}
        res_figures_lst = []
        res_dict['figures'] = res_figures_lst

        for fig in self.figures:
            res_figures_lst.append(fig.to_object())

        return res_dict

