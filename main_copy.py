import numpy as np
from numpy.lib.npyio import load
import open3d as o3d
import os
import matplotlib.pyplot as plt
from source.common import *
from source.settings import MEDIA_DIR, BASE_DIR
from time import sleep


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
        print(self.cur_ind)
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


stereo_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_stereo')
tof_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_tof')

fns_stereo = get_clouds_names_from_dir(stereo_dir)
fns_tof = get_clouds_names_from_dir(tof_dir)
fns_both = get_clouds_names_intersection_from_two_dir(stereo_dir, tof_dir)


pts_tof = [
    load_cloud(fn)
    for fn in fns_tof
]

VisCloudArr(pts_tof, .1).mult_show()