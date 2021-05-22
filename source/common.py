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

def show_pts_mtx(pts, mm):
    fig1 = plt.figure()
    axy = fig1.add_subplot()
    axy.set_xlim(mm[0])
    axy.set_ylim(mm[1])

    s_x = mm[0]
    s_y = mm[1]

    mtx = get_mtx_rop(pts, mm)
    mtx_size = mtx.shape

    # print(mtx)

    for i in range(mtx_size[0]):
        for j in range(mtx_size[1]):
            if mtx[i, j] == 1:
                x = i / mtx_size[0] * (s_x[1] - s_x[0]) + s_x[0]
                y = j / mtx_size[1] * (s_y[1] - s_y[0]) + s_y[0]
                axy.scatter(x, y, marker='s', c=0, linewidths=.01)


    axy.set_xlabel('X')
    axy.set_ylabel('Y')


    plt.show()

def show_pts(pts, mm):
    fig1 = plt.figure()
    fig2 = plt.figure()
    fig3 = plt.figure()
    axy = fig1.add_subplot()
    axy.set_xlim(mm[0])
    axy.set_ylim(mm[1])
    axz = fig2.add_subplot()
    axz.set_xlim(mm[0])
    axz.set_ylim(mm[2])
    ayz = fig3.add_subplot()
    ayz.set_xlim(mm[1])
    ayz.set_ylim(mm[2])

    for i, [xs, ys, zs] in enumerate(pts):
        if rnd.randint(0, 800) == 228:
            axy.scatter(xs, ys, marker='o')
            axz.scatter(xs, zs, marker='o')
            ayz.scatter(ys, zs, marker='o')

    axy.set_xlabel('X')
    axy.set_ylabel('Y')
    axz.set_xlabel('X')
    axz.set_ylabel('Z')
    ayz.set_xlabel('Y')
    ayz.set_ylabel('Z')

    plt.show()

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
    print(file_path)
    return np.asarray(o3d.io.read_point_cloud(file_path).points)


def get_clouds_names_from_dir(dir_path):
    _dir_path = os.path.join(BASE_DIR, dir_path)

    cloud_files_names = sorted([os.path.join(_dir_path, f) for f in
        os.listdir(_dir_path)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(_dir_path, f))
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

class m_visualization_with_key:

    def __init__(self, pts_lst, smooth = 1):
        self.smooth = smooth
        self.pts_lst = pts_lst
        self.pcds = [self.load_pts(pts) for pts in pts_lst]
        self.cur_ind = 0


        key_to_callback = {}
        key_to_callback[ord("D")] = self.next
        key_to_callback[ord("A")] = self.prev

        o3d.visualization.draw_geometries_with_key_callbacks(
            [self.pcds[self.cur_ind]], key_to_callback,

        )


    def load_pts(self, pts):
        if self.smooth != 1:
            pts = pts[[rnd.random() <= self.smooth for i in range(len(pts))]]

        cur_fn_path = os.path.join(MEDIA_DIR, 'point_cloud_train', 'test')
        cur_fn = os.path.join(cur_fn_path, 'test.pcd')
        if not os.path.isdir(cur_fn_path):
            os.mkdir(cur_fn_path)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud(cur_fn, pcd)

        return o3d.io.read_point_cloud(cur_fn)


    def refresh(self, vis):
        vis.clear_geometries()
        vis.add_geometry(self.pcds[self.cur_ind])

    def next(self, vis):
        if self.cur_ind < len(self.pcds) - 1:
            self.cur_ind += 1
        self.refresh(vis)
        return False

    def prev(self, vis):
        if self.cur_ind > 0:
            self.cur_ind -= 1
        self.refresh(vis)
        return False

    def load_all(self, vis):
        vis.clear_geometries()
        vis.add_geometry(self.pcds[0])
        vis.add_geometry(self.pcds[1])


    # def up(self, vis):
    #     self.pts_lst[0][..., 2] += .1
    #     self.pcds[0] = self.load_pts(self.pts_lst[0])
    #     self.load_all(vis)
    #     return False

    def both(self, vis):
        self.load_all(vis)
        return False
