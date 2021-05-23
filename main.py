#%%

import numpy as np
from numpy.lib.npyio import load
import open3d as o3d
import os
import matplotlib.pyplot as plt
from source.common import *
from source.settings import MEDIA_DIR, BASE_DIR
from time import sleep
from source.sensers.pcd_sens.sens import Sensor

stereo_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_stereo')
tof_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_tof')

fns_stereo = get_clouds_names_from_dir(stereo_dir)
fns_tof = get_clouds_names_from_dir(tof_dir)
fns_both = get_clouds_names_intersection_from_two_dir(stereo_dir, tof_dir)

# pts_both = load_intersection_cloud_points_from_two_dir(stereo_dir, tof_dir)


for fn in fns_tof[10:12]:
    print(fn)
    print(Sensor(fn).proc())

#%%

reses = [
    2, 2, 1, 2, 2, 2, 1, 2, 1, 1,
    1, 2, 1, 0, 0, 0, 2, 2, 2, 2,
    2, 1, 1, 0, 0, 2, 2, 2, 1, 0,
    2, 0, 0, 2, 2, 2, 1, 0, 2, 2
]

#%%
from source.sensers.figure import Figure
print(Figure().to_object())
