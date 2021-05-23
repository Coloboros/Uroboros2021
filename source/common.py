"""
Недокументируемая часть

Помойка полезных функций, в основном
для визуализации для анализа данных
для зарождения различных идей
для реальзации данного кейса
для решения сложной для нас
задачи для улучшения своих
качеств как специалистов своих ролей
"""
import numpy as np
import open3d as o3d
import random as rnd
import os
from .settings import BASE_DIR, MEDIA_DIR
import matplotlib.pyplot as plt
from scipy import stats
from time import sleep


def get_mtx_top(pts, mm, mtx_size=(64, 56, )):
    mtx = np.zeros(mtx_size)

    s_x = mm[0]
    s_y = mm[1]

    for i, [xs, ys, zs] in enumerate(pts):
        x = int(((xs - s_x[0]) / (s_x[1] - s_x[0])) * mtx_size[0])
        y = int(((ys - s_y[0]) / (s_y[1] - s_y[0])) * mtx_size[1])
        mtx[x, y] = 1

    return mtx

def get_min_max(pts):
    res = [
        (np.min(pts[..., i]) * 1.05, np.max(pts[..., i]) * 1.05, )
        for i in range(3)
    ]
    return res

def dtch(cld, graunded=False):
    return detach_train(detach_ground(cld)[int(graunded)])

def detach_ground(pts):
    mode_z = int(stats.mode(pts[..., 2])[0])
    res = (pts[pts[..., 2] < mode_z], pts[pts[..., 2] >= mode_z], )
    return res

def detach_train(pts):
    mode_y = int(stats.mode(pts[..., 1])[0])
    res = (pts[pts[..., 1] < mode_y], pts[pts[..., 1] >= mode_y], )
    return res



def load_cloud(file_path):
    return np.asarray(o3d.io.read_point_cloud(file_path).points)


def get_clouds_names_from_dir(dir_path):
    """
    Получение имён интересующих файлов из директории
    """
    _dir_path = os.path.join(BASE_DIR, dir_path)

    cloud_files_names = sorted([os.path.join(_dir_path, f) for f in
        os.listdir(_dir_path)
        if (f.endswith('.pcd') and os.path.isfile(os.path.join(_dir_path, f))) or
           (f.endswith('.pcd.zip') and os.path.isfile(os.path.join(_dir_path, f)))
    ])

    return cloud_files_names



def get_clouds_names_intersection_from_two_dir(dir_path1, dir_path2):
    _dir_path1 = os.path.join(BASE_DIR, dir_path1)
    _dir_path2 = os.path.join(BASE_DIR, dir_path2)

    cloud_files_names1 = sorted([os.path.join(_dir_path1, f) for f in
        os.listdir(_dir_path1)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(_dir_path1, f))
    ])

    cloud_files_names2 = sorted([os.path.join(_dir_path2, f) for f in
        os.listdir(_dir_path2)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(_dir_path2, f))
    ])

    fns_both = [(s, t, )
        for s in cloud_files_names1 for t in cloud_files_names2
        if os.path.basename(s).split('_')[-1] ==
           os.path.basename(t).split('_')[-1]
    ]

    return fns_both

def load_cloud_points_from_dir(dir_path):

    _dir_path = os.path.join(BASE_DIR, dir_path)

    cloud_files_names = sorted([os.path.join(_dir_path, f) for f in
        os.listdir(_dir_path)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(_dir_path, f))
    ])

    points = [
        np.asarray(o3d.io.read_point_cloud(cfn).points)
        for cfn in cloud_files_names
    ]

    return points

def load_intersection_cloud_points_from_two_dir(dir_path1, dir_path2):
    _dir1_path = os.path.join(BASE_DIR, dir_path1)
    _dir2_path = os.path.join(BASE_DIR, dir_path2)

    cloud_files_names1 = [os.path.join(_dir1_path, f) for f in
        os.listdir(_dir1_path)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(_dir1_path, f))
    ]
    cloud_files_names2 = [os.path.join(_dir2_path, f) for f in
        os.listdir(_dir2_path)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(_dir2_path, f))
    ]

    fns_both = [(s, t, )
        for s in cloud_files_names1 for t in cloud_files_names2
        if os.path.basename(s).split('_')[-1] ==
           os.path.basename(t).split('_')[-1]
    ]

    pts_sens = [
        np.concatenate((
            np.asarray(o3d.io.read_point_cloud(s).points),
            np.asarray(o3d.io.read_point_cloud(t).points)
        ), axis=0)
        for s, t in fns_both
    ]
    return pts_sens

def m_visualization(pts, smooth = 1, state=None):
    if smooth != 1:
        pts = pts[[rnd.random() <= smooth for i in range(len(pts))]]

    cur_fn_path = os.path.join(MEDIA_DIR, 'point_cloud_train', 'test')
    if not os.path.isdir(cur_fn_path):
        os.mkdir(cur_fn_path)

    cur_fn = os.path.join(cur_fn_path, 'test.pcd')

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    o3d.io.write_point_cloud(cur_fn, pcd)

    # pcd_load = o3d.io.read_point_cloud(cur_fn)
    # o3d.visualization.draw_geometries([pcd_load])


    vis = o3d.visualization.Visualizer()
    vis.create_window()

    geometry = o3d.io.read_point_cloud(cur_fn)
    vis.add_geometry(geometry)
    if state == 'tof':
        vc = vis.get_view_control()
        vc.rotate(0,500)
        vc.rotate(250,0)
        vc.rotate(0,250)
        vis.run()
    else:
        vis.run()

def m_visualization_all(pts_list, smooth = 1, state=None):

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for pts in pts_list:
        if smooth != 1:
            pts = pts[[rnd.random() <= smooth for i in range(len(pts))]]

        cur_fn_path = os.path.join(MEDIA_DIR, 'point_cloud_train', 'test')
        if not os.path.isdir(cur_fn_path):
            os.mkdir(cur_fn_path)

        cur_fn = os.path.join(cur_fn_path, 'test.pcd')

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud(cur_fn, pcd)

        # pcd_load = o3d.io.read_point_cloud(cur_fn)
        # o3d.visualization.draw_geometries([pcd_load])



        geometry = o3d.io.read_point_cloud(cur_fn)
        vis.add_geometry(geometry)
        if state == 'tof':
            vc = vis.get_view_control()
            vc.rotate(0,500)
            vc.rotate(250,0)
            vc.rotate(0,250)
            vis.poll_events()
            sleep(0.1)


def show_pts(pts, mm):
    fig1 = plt.figure()
    axy = fig1.add_subplot()
    axy.set_xlim(mm[0])
    axy.set_ylim(mm[2])

    # print(mtx)
    max_pt_count = 2000
    for i, [xs, ys, zs] in enumerate(pts):
        if rnd.random() < max_pt_count / len(pts):
            axy.scatter(xs, zs, marker='s', c=0, linewidths=.01)

    axy.set_xlabel('X')
    axy.set_ylabel('Z')


    plt.show()

def get_mtx_left(pts, mm, mtx_size=(64, 56, )):
    mtx = np.zeros(mtx_size)

    s_y = mm[1]
    s_z = mm[2]

    for i, [xs, ys, zs] in enumerate(pts):
        y = int(((ys - s_y[0]) / (s_y[1] - s_y[0])) * mtx_size[0])
        z = int(((zs - s_z[0]) / (s_z[1] - s_z[0])) * mtx_size[1])
        mtx[y, z] += 1

    return mtx

def filter_mtx(mtx, sensitivity = 3):
    n_mtx = mtx.copy() * 0
    for i in range(1, mtx.shape[0] - 1):
        for j in range(1, mtx.shape[1] - 1):
            if mtx[i, j] > sensitivity:
                n_mtx[i, j] = mtx[i, j]
    return n_mtx

def get_mtx_front(pts, mm, mtx_size=(64, 56, )):
    mtx = np.zeros(mtx_size)

    s_x = mm[0]
    s_z = mm[2]
    for i, [xs, ys, zs] in enumerate(pts):
        x = int(((xs - s_x[0]) / (s_x[1] - s_x[0])) * mtx_size[0])
        z = int(((zs - s_z[0]) / (s_z[1] - s_z[0])) * mtx_size[1])
        mtx[x, z] += 1

    mtx = np.flip(mtx, 1)
    return mtx


def show_pts_mtx(pts, mm, mtx_size=(64, 56, )):
    fig1 = plt.figure()
    axy = fig1.add_subplot()
    axy.set_xlim(mm[0])
    axy.set_ylim(mm[2])

    s_x = mm[0]
    s_z = mm[2]

    mtx = get_mtx_front(pts, mm, mtx_size)
    mtx = filter_mtx(mtx)
    mtx_size = mtx.shape

    start_ind = 2
    end_ind = start_ind
    height_wall = end_ind - start_ind + 1
    while np.sum(mtx[:, end_ind] != 0) / mtx.shape[1] > .7:
        end_ind += 1

    intresting_rows = []
    res = 0

    for i in range(1, mtx.shape[0] - 1):
        is_intr = 1 - np.sum(mtx[i, start_ind:end_ind+1] != 0) /\
                  (end_ind - start_ind + 1)
        if is_intr > .4:
            intresting_rows.append(i)
    if len(intresting_rows) > 1:
        intresting_rows = [
            i for i in range(
                min(intresting_rows),
                max(intresting_rows) + 1
            )
        ]

        intr_mm = (
            min(intresting_rows),
            max(intresting_rows)
        )

        count_pts_under_wall_in_door = \
            np.sum(mtx[intr_mm[0]:intr_mm[1]+1, end_ind+1:] != 0)
        # total_pts_under_wall_in_door = \
        #     (intr_mm[1] - intr_mm[0] + 1) * (mtx.shape[1] - end_ind + 1)
        if count_pts_under_wall_in_door > 5:
            res = 2
        else:
            res = 1


    mtx[:, start_ind] += 100
    mtx[:, end_ind] += 100

    for i in intresting_rows:
        mtx[i, :] += 4

    # print(mtx)

    for i in range(mtx_size[0]):
        for j in range(mtx_size[1]):
            if mtx[i, j] > 0:
                x = i / mtx_size[1] * (s_x[1] - s_x[0]) + s_x[0]
                z = j / mtx_size[0] * (s_z[1] - s_z[0]) + s_z[0]
                val = int(mtx[i, j])
                res_col = hex(255 - val if val < 256 else 0)[2:]
                if len(res_col) == 1:
                    res_col = '0' + res_col
                # if val < 100:
                #     res_col = f'#{res_col}0000'
                # else:
                #     res_col = f'#00{res_col}00'
                res_col = f'#00{res_col}00'
                axy.scatter(
                    x, z, marker='s',
                    c=res_col,
                    linewidths=1
                )


    axy.set_xlabel('X')
    axy.set_ylabel('Z')


    plt.show()

class VisCloud:
    def __init__(self,cloud,smooth = 1):
        self.smooth = smooth
        self.cloud = cloud
        self.smoothed_cloud = self.load_pts(self.cloud)

    def load_pts(self, pts):
        if self.smooth != 1:
            pts = pts[[rnd.random() <= self.smooth for i in range(len(pts))]]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        return pcd

    def show(self):
        o3d.visualization.draw_geometries([self.smoothed_cloud])

class VisCloudArr(VisCloud):
    def __init__(self,clouds, smooth = 1):
        super().__init__(clouds[0],smooth)
        self.cur_ind = 0
        self.clouds = clouds

        self.key_to_callback = {}
        self.key_to_callback[ord("D")] = self.next
        self.key_to_callback[ord("A")] = self.prev

    def mult_show(self):
        o3d.visualization.draw_geometries_with_key_callbacks(
            [self.load_pts(self.clouds[self.cur_ind])]
            ,self.key_to_callback
        )

    def refresh(self, vis):
        vis.clear_geometries()
        vis.add_geometry(self.load_pts(self.clouds[self.cur_ind]))
        vc = vis.get_view_control()
        vc.rotate(0,500)
        vc.rotate(0,250)

    def next(self, vis):
        if self.cur_ind < len(self.clouds) - 1:
            self.cur_ind += 1
        self.refresh(vis)
        return False

    def prev(self, vis):
        if self.cur_ind > 0:
            self.cur_ind -= 1
        self.refresh(vis)
        return False
