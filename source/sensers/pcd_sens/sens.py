import os
import numpy as np
import open3d as o3d
from scipy import stats
from zipfile import ZipFile

from source.sensers.interfaces import ISens
from source.sensers.out_data import OutData
from source.sensers.figure import Figure
from source.settings import MEDIA_DIR


door_code2str = {
    0: 'closed',
    1: 'opened_without_people_in_door',
    2: 'opened_with_people_in_door'
}


class Sensor(ISens):
    def __init__(self, f_name:str):
        assert os.path.isfile(f_name)
        self.f_name = f_name

    def proc(self):
        # Загрузка точек из файла с названием self.f_name
        pts = self._load_points_cloud(self.f_name)
        # Выделение нужных точек, отбразывая остальные
        pts = self._dtch(pts)

        # Определение размера матрицы
        # Слишком большие значения дадут плохой результат
        # потому-что, точек в каждой ячейки будет мало
        # и ячейки могут быть разбросанны через одну, что
        # нам совершенно не надо, тк нам важна плотность
        # Слишком маленькие значения будут так же давать
        # плохой результат, тк сложно будет различить
        # что-то
        # На соотношения сторон влияет только
        # ширина(те ось x) получаемых точек
        # и высота(те ось z) тех же точек
        mtx_size=(32, 28, )
        mm = self._get_min_max(pts)

        # получение матрицы проекции из точек
        mtx = self._get_mtx_front(pts, mm, mtx_size)
        mtx = self._filter_mtx(mtx)
        mtx_size = mtx.shape

        # нахождение высоты вагона до окон?
        start_ind = 2
        end_ind = start_ind
        height_wall = end_ind - start_ind + 1
        while np.sum(mtx[:, end_ind] != 0) / mtx.shape[1] > .7:
            end_ind += 1

        # конечное состояние двери
        door_code = -1
        # для индекса строк матрицы что находятся на двери
        intresting_rows = []

        # их нахождение
        for i in range(1, mtx.shape[0] - 1):
            is_intr = 1 - np.sum(mtx[i, start_ind:end_ind+1] != 0) / height_wall
            if is_intr > .4:
                intresting_rows.append(i)

        # просмотры случаев
        if len(intresting_rows) <= 1:
            # дверь не найденна
            door_code =  0
        else:
            # определение есть ли в двери объекты
            # лишь на уровне окон
            intr_mm = (
                min(intresting_rows),
                max(intresting_rows)
            )

            intresting_rows = [
                i for i in range(
                    intr_mm[0], intr_mm[1] + 1
                )
            ]

            count_pts_under_wall_in_door = \
                np.sum(mtx[intr_mm[0]:intr_mm[1]+1, end_ind+1:] != 0)


            door_code = int(count_pts_under_wall_in_door > 8) + 1

        # Запись результатов
        res_out = OutData(self.f_name)

        n_figure = Figure(door=door_code2str[door_code])
        res_out.add_figure(n_figure)

        return res_out

    def _load_points_cloud(self, file_path:str):
        """
        Загрузка точек3D из файла
        """
        if file_path.endswith('.pcd.zip'):
            tmp_dir_path = os.path.join(MEDIA_DIR, 'tmp')

            with ZipFile(file_path, 'r') as zipObj:
                zipObj.extractall(path=tmp_dir_path)

            f_name = os.path.split(file_path)[-1][:-4]
            n_f_path = os.path.join(tmp_dir_path, f_name)
            if os.path.isfile(n_f_path):
                res = np.asarray(o3d.io.read_point_cloud(n_f_path).points)
                os.remove(n_f_path)
                return res

            raise Exception("Can't read saved file")

        elif file_path.endswith('.pcd'):
            return np.asarray(o3d.io.read_point_cloud(file_path).points)

        raise Exception("file format doesn't support")

    def _get_min_max(self, pts):
        """
        Получение минимальных и максимальных значений всех осей
        из точек3D
        """
        res = [
            (np.min(pts[..., i]) * 1.05, np.max(pts[..., i]) * 1.05, )
            for i in range(3)
        ]
        return res

    def _filter_mtx(self, mtx, sensitivity = 3):
        """
        Обнулить элементы матрицы, в которые попало точек не больше
        чем параметр sensitivity
        """
        n_mtx = mtx.copy() * 0
        for i in range(1, mtx.shape[0] - 1):
            for j in range(1, mtx.shape[1] - 1):
                if mtx[i, j] > sensitivity:
                    n_mtx[i, j] = mtx[i, j]
        return n_mtx

    def _get_mtx_front(self, pts, mm, mtx_size=(64, 56, )):
        """
        Получение матрицы проекции точек с размером mtx_size
        на плоскость oXZ
        """
        mtx = np.zeros(mtx_size)

        s_x = mm[0]
        s_z = mm[2]
        for i, [xs, ys, zs] in enumerate(pts):
            x = int(((xs - s_x[0]) / (s_x[1] - s_x[0])) * mtx_size[0])
            z = int(((zs - s_z[0]) / (s_z[1] - s_z[0])) * mtx_size[1])
            mtx[x, z] += 1

        mtx = np.flip(mtx, 1)
        return mtx


    def _dtch(self, cld, graunded=False):
        """
        Отделить точки лежащие немного выше пола, и немного
        перед стенкой поезда
        """
        return self._detach_train(self._detach_ground(cld)[int(graunded)])[0]

    def _detach_ground(self, pts):
        """
        Отделить точки лежащие немного выше пола и ниже
        """
        mode_z = int(stats.mode(pts[..., 2])[0])
        res = (pts[pts[..., 2] < mode_z], pts[pts[..., 2] >= mode_z], )
        return res

    def _detach_train(self, pts):
        """
        Отделить точки лежащие немного перед стенкой поезда и после
        """
        mode_y = int(stats.mode(pts[..., 1])[0])
        res = (pts[pts[..., 1] < mode_y], pts[pts[..., 1] >= mode_y], )
        return res


__all__ = ['Sensor']
