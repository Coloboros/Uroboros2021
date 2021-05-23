import os
import json

from source.sensers.figure import Figure
from source.settings import BASE_DIR

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

    def save_file(self, f_path:str=None):
        """
        Сохранить данный объект со всеми своими данными по данному пути
        """
        f_path = f_path if f_path != None else self._get_out_file_path()
        f_dir, _ = os.path.split(f_path)
        self._mkdir(f_dir)

        with open(f_path, 'w') as outfile:
            data = self.to_object()
            json.dump(data, outfile)

    def _mkdir(self, dir_path):
        """
        Создание конечной директории
        """
        rel_f_dir = dir_path.replace(BASE_DIR, '')

        f_dir_lst = rel_f_dir.split(os.path.sep)
        if f_dir_lst[0] == '':
            f_dir_lst = f_dir_lst[1:]

        for i in range(len(f_dir_lst)):
            cur_dir = os.path.join(BASE_DIR, *f_dir_lst[:i+1])
            if not os.path.isdir(cur_dir):
                os.mkdir(cur_dir)


    def _get_out_file_path(self):
        """
        Получить путь к out-файлу
        """
        assert self.f_name != None
        f_dir, f_name = os.path.split(self.f_name)
        if f_name.endswith('.zip'):
            f_name = f_name[:-4]
        return os.path.join(f_dir, 'out', f_name + '.json')


    def to_object(self) -> dict:
        """
        Функция возвращающая объект геометрии как словарь
        """
        res_dict = {}
        res_figures_lst = []
        res_dict['figures'] = res_figures_lst

        for fig in self.figures:
            res_figures_lst.append(fig.to_object())

        return res_dict


__all__ = ['OutData']
