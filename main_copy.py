#%%

import numpy as np
from numpy.lib.npyio import load
import open3d as o3d
import os
import random as rnd
from scipy import stats
import matplotlib.pyplot as plt

import multiprocessing

#%%

def cloud_read(data):
    if type(data) is tuple:
        return np.concatenate((
            np.asarray(o3d.io.read_point_cloud(data[0]).points),
            np.asarray(o3d.io.read_point_cloud(data[1]).points)
        ), axis=0)
    else:
        return np.asarray(o3d.io.read_point_cloud(data).points)

def dtch(cld):
    return np.asarray(detach_train(detach_ground(cld)[0]))

def detach_ground(pts):
    mode_z = int(stats.mode(pts[..., 2])[0])
    res = (pts[pts[..., 2] < mode_z], pts[pts[..., 2] >= mode_z], )
    return res

def detach_train(pts):

    mode_y = int(stats.mode(pts[..., 1])[0])
    res = (pts[pts[..., 1] < mode_y], pts[pts[..., 1] >= mode_y], )
    return res

if __name__ == '__main__':
    stereo_dir = os.path.join('media', 'point_cloud_train', 'clouds_stereo')
    tof_dir = os.path.join('media', 'point_cloud_train', 'clouds_tof')

    fns_stereo = [os.path.join(stereo_dir, f) for f in
        os.listdir(stereo_dir)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(stereo_dir, f))
    ]
    fns_tof = [os.path.join(tof_dir, f) for f in
        os.listdir(tof_dir)
        if f.endswith('.pcd') and os.path.isfile(os.path.join(tof_dir, f))
    ]

    fns_both = [(s, t, )
        for s in fns_stereo for t in fns_tof
        if s.split(os.path.sep)[-1].split('_')[-1] ==
        t.split(os.path.sep)[-1].split('_')[-1]
    ]

    with multiprocessing.Pool(multiprocessing.cpu_count()) as p:
        pts_sens = p.map(cloud_read,fns_tof)
    # pts_sens = [
    #     np.concatenate((
    #         np.asarray(o3d.io.read_point_cloud(s).points),
    #         np.asarray(o3d.io.read_point_cloud(t).points)
    #     ), axis=0)
    #     for s, t in fns_both
    # ]

#%%

    def m_visualization(pts, smooth = 1):
        if smooth != 1:
            pts = pts[[rnd.random() <= smooth for i in range(len(pts))]]

        cur_fn_path = os.path.join('media', 'point_cloud_train', 'test')
        cur_fn = os.path.join(cur_fn_path, 'test.pcd')
        if not os.path.isdir(cur_fn_path):
            os.mkdir(cur_fn_path)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        # o3d.io.write_point_cloud(cur_fn, pcd)

        # # Load saved point cloud and visualize it
        # pcd_load = o3d.io.read_point_cloud(cur_fn)
        o3d.visualization.draw_geometries([pcd])   #[pcd_load])

    class Vis_Cloud:
        def __init__(self,cloud,smooth = 1):
            self.smooth = smooth
            self.cloud = cloud[0]
            self.smoothed_cloud = self.load_pts(self.cloud)
            #print(self.smoothed_cloud)

        def load_pts(self, pts):
            #print(pts)
            if self.smooth != 1:
                pts = pts[[rnd.random() <= self.smooth for i in range(len(pts))]]

            # cur_fn_path = os.path.join('media', 'point_cloud_train', 'test')
            # cur_fn = os.path.join(cur_fn_path, 'test.pcd')
            # if not os.path.isdir(cur_fn_path):
            #     os.mkdir(cur_fn_path)

            #print(pts)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)

            return pcd

        def show(self):
            o3d.visualization.draw_geometries([self.smoothed_cloud])

    class Vis_CloudArr(Vis_Cloud):
        def __init__(self,clouds, smooth = 1):
            super().__init__(clouds[0],smooth)
            self.cur_ind = 0
            self.clouds = clouds

            self.key_to_callback = {}
            self.key_to_callback[ord("D")] = self.next
            self.key_to_callback[ord("A")] = self.prev

        def mult_show(self):
            o3d.visualization.draw_geometries_with_key_callbacks([self.load_pts(self.clouds[self.cur_ind][0])],self.key_to_callback)

        def refresh(self, vis):
            vis.clear_geometries()
            vis.add_geometry(self.load_pts(self.clouds[self.cur_ind][0]))
            vc = vis.get_view_control()
            vc.rotate(0,500)
            vc.rotate(250,0)
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

    #%%

    #%%

    with multiprocessing.Pool(multiprocessing.cpu_count()) as p:
        cur_pts = p.map(dtch,pts_sens)
    viewer = Vis_CloudArr(cur_pts,.1)
    viewer.mult_show()

    #%%

    cur_pts = np.asarray(pts_sens[0])
    cur_pts_without_ground, ground_pts = detach_ground(cur_pts)

    m_visualization(cur_pts, .5)
    m_visualization(cur_pts_without_ground, .5)

    #%%
    for i in range(10):
        print(np.asarray(o3d.io.read_point_cloud(fns_both[i][0]).points).shape)
        print(np.asarray(o3d.io.read_point_cloud(fns_both[i][1]).points).shape)



    #%%

    def show_pts(pts):
        fig1 = plt.figure()
        fig2 = plt.figure()
        fig3 = plt.figure()
        axy = fig1.add_subplot()
        axz = fig2.add_subplot()
        ayz = fig3.add_subplot()

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

    #cld = o3d.geometry.PointCloud()
    #cld.points = o3d.utility.Vector3dVector(cur_pts_without_ground)
    #cur_pts = np.asarray(cld.points)
    show_pts(cur_pts_without_ground)
    #without_ground, _ = detach_ground(cur_pts)
    #show_pts(without_ground)

    #%%
    m_visualization(cur_pts, 1 / 300)
    m_visualization(without_ground, 1 / 300)