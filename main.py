#%%

import numpy as np
from numpy.lib.npyio import load
import open3d as o3d
import os
import matplotlib.pyplot as plt
from source.common import *
from source.settings import MEDIA_DIR, BASE_DIR
from time import sleep

#%%
stereo_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_stereo')
tof_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_tof')

fns_stereo = get_clouds_names_from_dir(stereo_dir)
fns_tof = get_clouds_names_from_dir(tof_dir)
fns_both = get_clouds_names_intersection_from_two_dir(stereo_dir, tof_dir)

# pts_both = load_intersection_cloud_points_from_two_dir(stereo_dir, tof_dir)

#%%

# cur_pts_list = [
#     load_cloud(f)
#     for f in fns_stereo[:10]
# ]

# cur_pts_list_without_ground = [
#     detach_ground(cur_pts)[0]
#     for cur_pts in cur_pts_list
# ]

# m_visualization_all(cur_pts_list, state='tof')

#%%
cur_pts = np.asarray(o3d.io.read_point_cloud(fns_both[1][0]).points)
m_visualization(cur_pts, .1)

#%%

cur_pts = np.asarray(o3d.io.read_point_cloud(fns_tof[1]).points)
cur_pts_without_ground, ground_pts = detach_ground(cur_pts)

m_visualization(cur_pts, .5)
m_visualization(cur_pts_without_ground, .5)

#%%
for i in range(10):
    print(np.asarray(o3d.io.read_point_cloud(fns_both[i][0]).points).shape)
    print(np.asarray(o3d.io.read_point_cloud(fns_both[i][1]).points).shape)


#%%
def get_mtx_front(pts, mm, mtx_size=(64, 56, )):
    mtx = np.zeros(mtx_size)

    s_x = mm[0]
    s_z = mm[2]

    for i, [xs, ys, zs] in enumerate(pts):
        x = int(((xs - s_x[0]) / (s_x[1] - s_x[0])) * mtx_size[0])
        z = int(((zs - s_z[0]) / (s_z[1] - s_z[0])) * mtx_size[1])
        mtx[x, z] = 1

    return mtx


def get_mtx_left(pts, mm, mtx_size=(64, 56, )):
    mtx = np.zeros(mtx_size)

    s_y = mm[1]
    s_z = mm[2]

    for i, [xs, ys, zs] in enumerate(pts):
        y = int(((ys - s_y[0]) / (s_y[1] - s_y[0])) * mtx_size[0])
        z = int(((zs - s_z[0]) / (s_z[1] - s_z[0])) * mtx_size[1])
        mtx[y, z] = 1

    return mtx

def filter(mtx):
    n_mtx = mtx.copy() * 0
    for i in range(1, mtx.shape[0] - 1):
        for j in range(1, mtx.shape[1] - 1):
            if mtx[i, j] == 1 and np.sum(mtx[i-1:i+2, j-1:j+2]) >= 4:
                n_mtx[i, j] = 1
    # while(np.sum(n_mtx[:,-2:]) < n_mtx.shape[1] * .1):
    #     n_mtx = n_mtx[:, :-1]
    # print(n_mtx.shape)
    return n_mtx

def show_pts_mtx(pts, mm, mtx_size=(64, 56, )):
    fig1 = plt.figure()
    axy = fig1.add_subplot()
    axy.set_xlim(mm[0])
    axy.set_ylim(mm[2])

    s_x = mm[0]
    s_z = mm[2]

    mtx = get_mtx_front(pts, mm, mtx_size)
    mtx = filter(mtx)
    mtx_size = mtx.shape

    # print(mtx)

    for i in range(mtx_size[0]):
        for j in range(mtx_size[1]):
            if mtx[i, j] == 1:
                x = i / mtx_size[1] * (s_x[1] - s_x[0]) + s_x[0]
                z = j / mtx_size[0] * (s_z[1] - s_z[0]) + s_z[0]
                axy.scatter(x, z, marker='s', c=0, linewidths=.01)


    axy.set_xlabel('X')
    axy.set_ylabel('Z')


    plt.show()

def process(pts, mm, mtx_size=(64, 56, )):
    front_mtx = filter(get_mtx_front(pts, mm, mtx_size))
    i = 0
    while(np.sum(front_mtx[:,-i]) < front_mtx.shape[1] * .1):
        i+=1

    wall_sum = np.sum(front_mtx[:,-i-7:-i+1]) / 7
    below_wall_sum = np.sum(front_mtx[:,-i-14:-i-6]) / 7
    print()
    print(i)
    print(wall_sum)
    print(below_wall_sum)
    # if wall_sum >

    print(front_mtx)


for i in range(40):
    print(i)
    cur_pts = np.asarray(o3d.io.read_point_cloud(fns_tof[i]).points)
    mm = get_min_max(cur_pts)
    handlel_layer, _ = dtch(cur_pts)

    # mtx_size = (15, 12)
    process(handlel_layer, mm)
    show_pts_mtx(handlel_layer, mm)
    m_visualization(handlel_layer)
    # show_pts_mtx(only_ground, mm)
    # show_pts_mtx(without_ground, mm)
# show_pts(without_ground)
# show_pts(only_ground)

#%%
cur_pts = np.asarray(o3d.io.read_point_cloud(fns_tof[21]).points)
m_visualization(cur_pts)

#%%
# m_visualization(cur_pts, 1 / 300)
# m_visualization(without_ground, 1 / 300)