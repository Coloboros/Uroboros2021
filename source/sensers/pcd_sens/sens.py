import numpy as np
import open3d as o3d
from scipy import stats

from source.sensers.interfaces import ISens
from source.sensers.out_data import OutData
from source.sensers.figure import Figure


door_code2str = {
    0: 'closed',
    1: 'opened_without_people_in_door',
    2: 'opened_with_people_in_door'
}

class Sensor(ISens):
    def __init__(self, f_name):
        self.f_name = f_name

    def proc(self):
        pts = self._load_cloud(self.f_name)
        pts, _ = self._dtch(pts)

        mtx_size=(32, 28, )
        mm = self._get_min_max(pts)

        mtx = self._get_mtx_front(pts, mm, mtx_size)
        mtx = self._filter_mtx(mtx)
        mtx_size = mtx.shape


        start_ind = 2
        end_ind = start_ind
        height_wall = end_ind - start_ind + 1
        while np.sum(mtx[:, end_ind] != 0) / mtx.shape[1] > .7:
            end_ind += 1

        door_code = -1
        intresting_rows = []

        for i in range(1, mtx.shape[0] - 1):
            is_intr = 1 - np.sum(mtx[i, start_ind:end_ind+1] != 0) / height_wall
            if is_intr > .4:
                intresting_rows.append(i)

        if len(intresting_rows) <= 1:
            door_code =  0
        else:
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


        res_out = OutData(self.f_name)

        n_figure = Figure(door=door_code2str[door_code])
        res_out.add_figure(n_figure)

        return res_out

    def _load_cloud(self, file_path):
        return np.asarray(o3d.io.read_point_cloud(file_path).points)

    def _get_min_max(self, pts):
        res = [
            (np.min(pts[..., i]) * 1.05, np.max(pts[..., i]) * 1.05, )
            for i in range(3)
        ]
        return res

    def _filter_mtx(self, mtx, sensitivity = 3):
        n_mtx = mtx.copy() * 0
        for i in range(1, mtx.shape[0] - 1):
            for j in range(1, mtx.shape[1] - 1):
                if mtx[i, j] > sensitivity:
                    n_mtx[i, j] = mtx[i, j]
        return n_mtx

    def _get_mtx_front(self, pts, mm, mtx_size=(64, 56, )):
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
        return self._detach_train(self._detach_ground(cld)[int(graunded)])

    def _detach_ground(self, pts):
        mode_z = int(stats.mode(pts[..., 2])[0])
        res = (pts[pts[..., 2] < mode_z], pts[pts[..., 2] >= mode_z], )
        return res

    def _detach_train(self, pts):
        mode_y = int(stats.mode(pts[..., 1])[0])
        res = (pts[pts[..., 1] < mode_y], pts[pts[..., 1] >= mode_y], )
        return res